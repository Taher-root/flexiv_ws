/**
 * @file rt_bridge.cpp
 * @brief The ROS half: FollowJointTrajectory and /joint_states over shm.
 *
 * Links rclcpp and nothing from the RDK, which is the point of the split: the
 * only RDK archive that works on this robot expects Flexiv's vendored Fast-DDS,
 * and rclcpp loads ROS 2's. See README.md.
 *
 * Deliberately thin. Validation, joint remapping and the execution state
 * machine live in traj_ingest.hpp and rt_executor.hpp, both unit-tested without
 * ROS, because this file is the one part of the package that cannot be compiled
 * or exercised away from a ROS installation.
 *
 * It serves `follow_joint_trajectory` in its own namespace, so running it as
 * `/left_arm` makes it a drop-in for aico2_left_arm_driver with no change to
 * aico2_moveit_config. The two are mutually exclusive: both the Python driver
 * and rt_server want the single RDK session the robot allows, so exactly one of
 * them runs at a time.
 *
 *   ros2 run aico2_rt_control rt_bridge --ros-args -r __ns:=/left_arm
 */

#include "aico2_rt_control/traj_ingest.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using namespace std::chrono_literals;

class RtBridge : public rclcpp::Node {
public:
    RtBridge() : rclcpp::Node("rt_bridge")
    {
        shm_name_ = declare_parameter<std::string>("shm_name", aico2_rt::kDefaultShmName);
        // Joint names in ROBOT VECTOR ORDER: external axes first, confirmed by
        // joint_map_probe. A goal may list them in any order; this is the order
        // the RDK expects them streamed in.
        joint_names_ = declare_parameter<std::vector<std::string>>("joint_names",
            {"AGV_Joint1", "AGV_Joint2", "Left_joint1", "Left_joint2", "Left_joint3",
                "Left_joint4", "Left_joint5", "Left_joint6", "Left_joint7"});
        const double rate = declare_parameter<double>("joint_state_rate_hz", 100.0);
        start_tolerance_ = declare_parameter<double>("start_tolerance", 0.05);
        rest_tolerance_ = declare_parameter<double>("rest_tolerance", 0.01);

        if (!OpenShm()) {
            throw std::runtime_error("could not attach to rt_server's shared memory");
        }
        if (joint_names_.size() != shm_->dof) {
            throw std::runtime_error("joint_names has " + std::to_string(joint_names_.size())
                + " entries but the server reports DoF " + std::to_string(shm_->dof));
        }

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
            [this] { PublishJointStates(); });

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(this,
            "follow_joint_trajectory",
            [this](const rclcpp_action::GoalUUID& u,
                std::shared_ptr<const FollowJointTrajectory::Goal> g) {
                return HandleGoal(u, g);
            },
            [this](const std::shared_ptr<GoalHandle>) {
                // Accept every cancel: the executor decelerates at ddq_max
                // rather than stopping dead, so this is always safe to honour.
                shm_->cancel_request.store(1u, std::memory_order_release);
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> h) {
                // Must return promptly, so execution runs on its own thread.
                std::thread{[this, h] { Execute(h); }}.detach();
            });

        RCLCPP_INFO(get_logger(), "attached to '%s', DoF %u, waist %s", shm_name_.c_str(),
            shm_->dof, shm_->control_waist ? "COMMANDED" : "pinned");
    }

    ~RtBridge() override
    {
        if (shm_ != nullptr) {
            munmap(shm_, sizeof(aico2_rt::Shm));
        }
        if (shm_fd_ >= 0) {
            close(shm_fd_);
        }
    }

private:
    bool OpenShm()
    {
        shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0600);
        if (shm_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "shm_open(%s): %s -- is rt_server running?",
                shm_name_.c_str(), std::strerror(errno));
            return false;
        }
        void* raw = mmap(nullptr, sizeof(aico2_rt::Shm), PROT_READ | PROT_WRITE, MAP_SHARED,
            shm_fd_, 0);
        if (raw == MAP_FAILED) {
            RCLCPP_ERROR(get_logger(), "mmap: %s", std::strerror(errno));
            return false;
        }
        shm_ = static_cast<aico2_rt::Shm*>(raw);
        if (shm_->magic != aico2_rt::kShmMagic || shm_->version != aico2_rt::kShmVersion) {
            RCLCPP_ERROR(get_logger(), "shared memory is not a v%u aico2_rt mapping",
                aico2_rt::kShmVersion);
            return false;
        }
        return true;
    }

    /** The RT heartbeat is the cycle counter, so a live server moves it ~1000
     *  times a second. Two identical reads 50 ms apart means nothing is driving
     *  the robot, and accepting a goal would strand the caller. */
    bool ServerAlive()
    {
        const std::uint64_t a = shm_->rt_heartbeat.load(std::memory_order_acquire);
        std::this_thread::sleep_for(50ms);
        return shm_->rt_heartbeat.load(std::memory_order_acquire) != a;
    }

    aico2_rt::IngestConfig Config() const
    {
        aico2_rt::IngestConfig c;
        c.dof = shm_->dof;
        c.n_external = shm_->n_external;
        c.allow_waist_motion = (shm_->control_waist != 0u);
        c.start_tolerance = start_tolerance_;
        c.rest_tolerance = rest_tolerance_;
        c.limits = shm_->limits;
        return c;
    }

    /**
     * @brief Convert a goal into robot vector order.
     * @return kNone, or why it cannot be used.
     *
     * MoveIt fills positions, velocities and accelerations; the quintic
     * resampler needs all three, so a goal missing velocities or accelerations
     * is refused rather than padded with zeros -- zeros would validate as
     * "starts at rest" and then be interpolated as though the plan really had
     * no velocity, which is a quietly wrong trajectory.
     */
    aico2_rt::RejectReason Convert(const trajectory_msgs::msg::JointTrajectory& jt,
        std::vector<aico2_rt::Point>& out)
    {
        std::vector<int> map;
        const auto why = aico2_rt::BuildJointMap(jt.joint_names, joint_names_, map);
        if (why != aico2_rt::RejectReason::kNone) {
            return why;
        }
        if (jt.points.empty() || jt.points.size() > aico2_rt::kMaxPoints) {
            return aico2_rt::RejectReason::kTooManyPoints;
        }
        out.assign(jt.points.size(), aico2_rt::Point{});
        for (std::size_t i = 0; i < jt.points.size(); ++i) {
            const auto& p = jt.points[i];
            if (p.positions.size() != jt.joint_names.size()
                || p.velocities.size() != jt.joint_names.size()
                || p.accelerations.size() != jt.joint_names.size()) {
                return aico2_rt::RejectReason::kDofMismatch;
            }
            out[i].t = rclcpp::Duration(p.time_from_start).seconds();
            for (std::size_t k = 0; k < map.size(); ++k) {
                const auto j = static_cast<std::size_t>(map[k]);
                out[i].q[j] = p.positions[k];
                out[i].dq[j] = p.velocities[k];
                out[i].ddq[j] = p.accelerations[k];
            }
        }
        return aico2_rt::RejectReason::kNone;
    }

    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ServerAlive()) {
            RCLCPP_ERROR(get_logger(), "rejecting: %s",
                aico2_rt::RejectReasonName(aico2_rt::RejectReason::kServerNotRunning));
            return rclcpp_action::GoalResponse::REJECT;
        }
        aico2_rt::Measured meas{};
        if (!aico2_rt::ReadMeasured(*shm_, meas)) {
            RCLCPP_ERROR(get_logger(), "rejecting: no coherent state from the server");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (meas.operational == 0u || meas.fault != 0u) {
            RCLCPP_ERROR(get_logger(), "rejecting: %s",
                aico2_rt::RejectReasonName(aico2_rt::RejectReason::kNotOperational));
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::vector<aico2_rt::Point> pts;
        auto why = Convert(goal->trajectory, pts);
        if (why == aico2_rt::RejectReason::kNone) {
            why = aico2_rt::ValidateTrajectory(
                pts.data(), static_cast<std::uint32_t>(pts.size()), meas.q, Config());
        }
        if (why != aico2_rt::RejectReason::kNone) {
            RCLCPP_ERROR(get_logger(), "rejecting goal: %s", aico2_rt::RejectReasonName(why));
            return rclcpp_action::GoalResponse::REJECT;
        }
        pending_ = std::move(pts);
        RCLCPP_INFO(get_logger(), "accepted goal: %zu points, %.2f s", pending_.size(),
            pending_.back().t);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void Execute(const std::shared_ptr<GoalHandle> handle)
    {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        std::vector<aico2_rt::Point> pts;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pts = std::move(pending_);
            pending_.clear();
        }
        if (pts.empty()) {
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            result->error_string = "no validated trajectory to execute";
            handle->abort(result);
            return;
        }

        const auto n = static_cast<std::uint32_t>(pts.size());
        const std::uint32_t slot = aico2_rt::PickFreeSlot(*shm_);
        aico2_rt::FillSlot(shm_->slots[slot], ++goal_counter_, pts.data(), n, shm_->dof);
        const std::uint64_t seq = aico2_rt::PublishSlot(*shm_, slot);

        // Adoption is one RT cycle away, so ~1 ms. A second of grace is
        // generous and still bounded.
        for (int i = 0; i < 1000 && shm_->adopted_seq.load() < seq; ++i) {
            std::this_thread::sleep_for(1ms);
        }
        if (shm_->adopted_seq.load() < seq) {
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            result->error_string = "the RT server did not adopt the trajectory";
            handle->abort(result);
            return;
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->joint_names = joint_names_;
        const double duration = pts.back().t;
        const auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::duration<double>(duration + 5.0);

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            if (handle->is_canceling()) {
                shm_->cancel_request.store(1u, std::memory_order_release);
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                result->error_string = "cancelled; decelerated to a stop";
                handle->canceled(result);
                return;
            }
            aico2_rt::RtStatus st{};
            aico2_rt::Measured meas{};
            if (aico2_rt::ReadStatus(*shm_, st) && aico2_rt::ReadMeasured(*shm_, meas)) {
                PublishFeedback(handle, feedback, st, meas);
                const auto state = static_cast<aico2_rt::ExecState>(st.exec_state);
                if (state == aico2_rt::ExecState::kFinished) {
                    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                    if (st.clamped != 0u) {
                        // Not a failure -- the motion completed -- but it means
                        // something was asked for that the limits forbid, which
                        // validation should have caught.
                        result->error_string = std::string("completed, but joint ")
                            + std::to_string(st.clamp_joint) + " clamped: "
                            + aico2_rt::ClampKindName(static_cast<aico2_rt::ClampKind>(
                                  st.clamp_kind));
                        RCLCPP_WARN(get_logger(), "%s", result->error_string.c_str());
                    }
                    handle->succeed(result);
                    return;
                }
                if (state == aico2_rt::ExecState::kAborted) {
                    result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                    result->error_string = "the RT server aborted: fault or cancel";
                    handle->abort(result);
                    return;
                }
                if (state == aico2_rt::ExecState::kRejected) {
                    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                    result->error_string = std::string("the RT server rejected it: ")
                        + aico2_rt::RejectReasonName(
                            static_cast<aico2_rt::RejectReason>(st.reject_reason));
                    handle->abort(result);
                    return;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "timed out waiting for the trajectory to finish";
        handle->abort(result);
    }

    void PublishFeedback(const std::shared_ptr<GoalHandle>& handle,
        const std::shared_ptr<FollowJointTrajectory::Feedback>& fb, const aico2_rt::RtStatus& st,
        const aico2_rt::Measured& meas)
    {
        const std::size_t n = joint_names_.size();
        fb->header.stamp = now();
        fb->desired.positions.assign(st.cmd_q, st.cmd_q + n);
        fb->desired.velocities.assign(st.cmd_dq, st.cmd_dq + n);
        fb->actual.positions.assign(meas.q, meas.q + n);
        fb->actual.velocities.assign(meas.dq, meas.dq + n);
        fb->error.positions.resize(n);
        fb->error.velocities.resize(n);
        for (std::size_t j = 0; j < n; ++j) {
            fb->error.positions[j] = st.cmd_q[j] - meas.q[j];
            fb->error.velocities[j] = st.cmd_dq[j] - meas.dq[j];
        }
        fb->desired.time_from_start = rclcpp::Duration::from_seconds(st.traj_time);
        handle->publish_feedback(fb);
    }

    void PublishJointStates()
    {
        aico2_rt::Measured meas{};
        if (!aico2_rt::ReadMeasured(*shm_, meas)) {
            return;  // a write was in flight; next tick will do
        }
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = joint_names_;
        const std::size_t n = joint_names_.size();
        msg.position.assign(meas.q, meas.q + n);
        msg.velocity.assign(meas.dq, meas.dq + n);
        msg.effort.assign(meas.tau, meas.tau + n);
        joint_state_pub_->publish(msg);
    }

    std::string shm_name_;
    std::vector<std::string> joint_names_;
    double start_tolerance_ = 0.05;
    double rest_tolerance_ = 0.01;
    int shm_fd_ = -1;
    aico2_rt::Shm* shm_ = nullptr;
    std::uint64_t goal_counter_ = 0;
    std::mutex mutex_;
    std::vector<aico2_rt::Point> pending_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        // A multi-threaded executor, because the action callbacks, the timer
        // and the feedback loop must not block one another.
        auto node = std::make_shared<RtBridge>();
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rt_bridge"), "%s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
