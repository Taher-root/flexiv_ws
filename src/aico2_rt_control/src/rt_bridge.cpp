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
 *
 * It serves two channels, which the RT loop treats as mutually exclusive:
 *
 *   follow_joint_trajectory   action. MoveIt's Plan & Execute.
 *   servo_joint_command       topic, trajectory_msgs/JointTrajectory. MoveIt
 *                             Servo and VR teleop, gated by set_teleop_mode.
 *
 * Both names match aico2_left_arm_driver's, deliberately: the Servo node's
 * command_out_topic and the teleop client's service call are then identical
 * whether the NRT Python driver or this bridge is the one running, so moving a
 * working teleop setup onto the RT path needs no configuration change at all.
 */

#include "aico2_rt_control/traj_ingest.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
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

// The servo target is stamped with aico2_rt::MonotonicSeconds() from
// shm_protocol.hpp -- the one clock both processes agree on. See its comment.

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
        // 200 Hz rather than 100: /joint_states is MoveIt Servo's only view of
        // where the arm is, so under a servo stream this rate is the slowest
        // link in the feedback loop. It costs nine doubles per message.
        const double rate = declare_parameter<double>("joint_state_rate_hz", 200.0);
        // Which joints a servo stream may command. The arm only by default:
        // MoveIt Servo runs on a planning group, and the waist is pinned
        // unless rt_server was started with --control-waist.
        servo_joint_names_ = declare_parameter<std::vector<std::string>>(
            "servo_joint_names", {"Left_joint1", "Left_joint2", "Left_joint3",
                                     "Left_joint4", "Left_joint5", "Left_joint6",
                                     "Left_joint7"});
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
        // Its own callback group, because entities created without one share
        // the node's default group and that group is mutually exclusive. With
        // /joint_states on the default group, set_teleop_mode(false) -- which
        // waits up to 5 s for the arm to finish braking -- would stop
        // publishing joint states for the whole wait. /joint_states is MoveIt
        // Servo's only view of where the arm is, so that is the one thing that
        // must keep running while something else is blocking.
        js_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
            [this] { PublishJointStates(); }, js_cb_group_);

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

        // The servo mask, resolved once. A name here that is not a robot joint
        // is a configuration error worth failing on rather than discovering as
        // an arm that will not move.
        for (const auto& name : servo_joint_names_) {
            const auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
            if (it == joint_names_.end()) {
                throw std::runtime_error("servo_joint_names contains '" + name
                    + "', which is not one of this server's joints");
            }
            servo_mask_ |= 1u << static_cast<std::uint32_t>(it - joint_names_.begin());
        }

        // A mutually-exclusive callback group, and it matters: this node runs
        // on a MultiThreadedExecutor, and HandleServoCommand is the servo
        // seqlock's only writer. Two of these callbacks running at once would
        // make it two writers, and a seqlock with two writers is not a seqlock
        // -- the counter would go even mid-update and the RT loop could read a
        // pose assembled from two different targets. A lock on the callback
        // would also work; keeping the group single-threaded costs nothing on
        // a path that does no waiting anyway.
        servo_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions servo_opts;
        servo_opts.callback_group = servo_cb_group_;
        servo_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "servo_joint_command", rclcpp::SensorDataQoS(),
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
                HandleServoCommand(*msg);
            },
            servo_opts);
        // Likewise its own group: this handler blocks for as long as the brake
        // takes, and it must not hold up the servo subscription or the timer
        // while it does.
        srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        teleop_srv_ = create_service<std_srvs::srv::SetBool>("set_teleop_mode",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                HandleSetTeleopMode(req, res);
            },
            rclcpp::ServicesQoS(), srv_cb_group_);

        // Gripper: command subscriber + state publisher. The gripper channel
        // is independent of the arm trajectory/servo path. Positive value =
        // grasp with that force [N]; zero = open fully; negative = move to
        // abs(value) width [m]. The convention is deliberately simple so a
        // joystick button can drive it.
        gripper_cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
            "gripper_command", 10,
            [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
                HandleGripperCommand(msg->data);
            });
        gripper_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "gripper_states", 10);

        RCLCPP_INFO(get_logger(), "attached to '%s', DoF %u, waist %s", shm_name_.c_str(),
            shm_->dof, shm_->control_waist ? "COMMANDED" : "pinned");
        // The first arm axis, not index 0: without --control-waist the leading
        // external axes are pinned to a 1e-3 envelope, so their jerk limit is
        // 0.03 rad/s^3 and logging it reads as "no jerk limit configured".
        const std::uint32_t arm0 =
            shm_->n_external < shm_->dof ? shm_->n_external : 0u;
        RCLCPP_INFO(get_logger(),
            "servo stream: %zu joints (mask 0x%x), arm jerk %.1f rad/s^3, timeout %.3f s,"
            " max jump %.3f rad",
            servo_joint_names_.size(), servo_mask_, shm_->servo_cfg.max_jerk[arm0],
            shm_->servo_cfg.timeout_sec, shm_->servo_cfg.max_jump_rad);
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

    /**
     * @brief Forward one streamed setpoint to the RT loop.
     *
     * Deliberately not validated the way a trajectory is. A trajectory is
     * checked once, up front, against rules that only make sense for a plan --
     * starts at the measured position, ends at rest, time strictly increasing.
     * A servo stream satisfies none of them by design: every sample is mid-
     * motion, there is no end, and there is no timing at all. So the checks
     * here are the ones that mean something per sample -- is this message
     * well-formed, and is it about joints this stream is allowed to move --
     * and everything else is the tracker's job, where it can be enforced
     * continuously instead of once: the joint limits, the velocity and
     * acceleration bounds, the jerk limit, and the glitch rejection.
     *
     * Dropping a message is never silent, but it is throttled: at a few hundred
     * hertz an unthrottled warning would be its own outage.
     */
    void HandleServoCommand(const trajectory_msgs::msg::JointTrajectory& msg)
    {
        if (shm_->servo_enable.load(std::memory_order_acquire) == 0u) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "ignoring servo command: teleop mode is off."
                " Call set_teleop_mode with data: true first.");
            return;
        }
        if (msg.points.empty()) {
            return;
        }
        // The last point, matching aico2_left_arm_driver: Servo sends one point
        // per tick, and where there are several the last is the furthest ahead.
        const auto& pt = msg.points.back();
        if (pt.positions.size() != msg.joint_names.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "ignoring servo command: %zu names but %zu positions",
                msg.joint_names.size(), pt.positions.size());
            return;
        }

        double q[aico2_rt::kMaxDof] = {};
        double dq[aico2_rt::kMaxDof] = {};
        const bool have_dq = pt.velocities.size() == msg.joint_names.size();
        std::uint32_t mask = 0u;
        for (std::size_t k = 0; k < msg.joint_names.size(); ++k) {
            const auto it = std::find(
                joint_names_.begin(), joint_names_.end(), msg.joint_names[k]);
            if (it == joint_names_.end()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "ignoring servo command: unknown joint '%s'",
                    msg.joint_names[k].c_str());
                return;
            }
            const auto j = static_cast<std::uint32_t>(it - joint_names_.begin());
            if ((servo_mask_ & (1u << j)) == 0u) {
                // A static configuration error, so refusing the whole message
                // is right: silently dropping the joint would move the arm
                // somewhere other than asked, every single tick.
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "ignoring servo command: joint '%s' is not in servo_joint_names",
                    msg.joint_names[k].c_str());
                return;
            }
            q[j] = pt.positions[k];
            if (have_dq) {
                dq[j] = pt.velocities[k];
            }
            mask |= 1u << j;
        }

        // Stamped on receipt, not from msg.header: the age is used to notice
        // the producer has stopped, which is a question about this bridge's own
        // clock. A producer's header stamp would need the two clocks
        // synchronised to mean anything, and a producer on another machine --
        // which is exactly what a VR headset's server is -- will not be.
        aico2_rt::WriteServoTarget(*shm_, q, have_dq ? dq : nullptr, mask, shm_->dof,
            have_dq, aico2_rt::MonotonicSeconds());
    }

    /**
     * @brief Turn the servo stream on or off, and report what actually happened.
     *
     * Enabling is refused while a trajectory is in motion. The executor guards
     * this too, but it guards it by ignoring the request, which from here would
     * look like success -- so the state is read back and the transition
     * confirmed rather than assumed. Disabling waits for the brake: the tracker
     * ramps down at the jerk limit and only then hands back to a hold, and
     * returning before that would let a caller start a trajectory into a
     * moving arm.
     */
    void HandleSetTeleopMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        aico2_rt::RtStatus st{};
        if (!aico2_rt::ReadStatus(*shm_, st)) {
            res->success = false;
            res->message = "no coherent state from the RT server";
            return;
        }
        const auto state = static_cast<aico2_rt::ExecState>(st.exec_state);

        if (req->data) {
            if (state == aico2_rt::ExecState::kRunning
                || state == aico2_rt::ExecState::kStopping) {
                res->success = false;
                res->message = "cannot enter teleop: a trajectory is running";
                return;
            }
            shm_->servo_enable.store(1u, std::memory_order_release);
            res->success = AwaitState(aico2_rt::ExecState::kServoing, true, 1.0);
            res->message = res->success ? "teleop enabled"
                                        : "the RT server did not enter servo mode";
            if (!res->success) {
                shm_->servo_enable.store(0u, std::memory_order_release);
            } else {
                RCLCPP_INFO(get_logger(), "teleop ENABLED (servo stream)");
            }
            return;
        }

        shm_->servo_enable.store(0u, std::memory_order_release);
        // Generous: the brake is bounded by ddq_max and max_jerk, so from full
        // speed on a slow axis it is a fraction of a second, but waiting is
        // always better than reporting a stop that has not happened.
        res->success = AwaitState(aico2_rt::ExecState::kServoing, false, 5.0);
        res->message = res->success ? "teleop disabled; decelerated to a hold"
                                    : "teleop disabled, but the arm has not reported a stop";
        RCLCPP_INFO(get_logger(), "teleop DISABLED (%s)", res->message.c_str());
    }

    /**
     * @brief Write a gripper command into shm for rt_server's non-RT thread.
     *
     * Convention: value > 0 → grasp with that force (N). value == 0 → open
     * fully at default velocity. value < 0 → move to abs(value) width (m).
     */
    void HandleGripperCommand(float value)
    {
        aico2_rt::GripperState gs{};
        if (!aico2_rt::ReadGripperState(*shm_, gs) || gs.ready == 0u) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "ignoring gripper command: no gripper configured on rt_server"
                " (start rt_server with --gripper NAME)");
            return;
        }
        aico2_rt::BeginGripperCmdWrite(*shm_);
        aico2_rt::GripperCommand& gc = shm_->gripper_cmd;
        if (value > 0.0f) {
            gc.cmd = static_cast<std::uint32_t>(aico2_rt::GripperCmd::kGrasp);
            gc.force = static_cast<double>(value);
            gc.width = 0.0;
            gc.velocity = 0.0;
        } else if (value < 0.0f) {
            gc.cmd = static_cast<std::uint32_t>(aico2_rt::GripperCmd::kMove);
            gc.width = static_cast<double>(-value);
            gc.velocity = gs.max_vel * 0.5;
            gc.force = gs.max_force;
        } else {
            gc.cmd = static_cast<std::uint32_t>(aico2_rt::GripperCmd::kMove);
            gc.width = gs.max_width;
            gc.velocity = gs.max_vel * 0.5;
            gc.force = gs.max_force;
        }
        aico2_rt::EndGripperCmdWrite(*shm_);
    }

    /** @brief Poll the published state until it does (or stops) matching. */
    bool AwaitState(aico2_rt::ExecState want, bool present, double timeout_sec)
    {
        const auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::duration<double>(timeout_sec);
        while (std::chrono::steady_clock::now() < deadline) {
            aico2_rt::RtStatus st{};
            if (aico2_rt::ReadStatus(*shm_, st)) {
                const bool is = static_cast<aico2_rt::ExecState>(st.exec_state) == want;
                if (is == present) {
                    return true;
                }
            }
            std::this_thread::sleep_for(2ms);
        }
        return false;
    }

    /**
     * @brief Accept almost everything, and let Execute explain any refusal.
     *
     * A REJECTED goal carries no result message in ROS 2 -- the caller learns
     * only that it was refused, and the reason exists solely in this node's
     * log. That made every failed goal a trip to another terminal. So the real
     * validation happens in Execute, which can abort with the detail in
     * error_string, where the caller and MoveIt both see it.
     *
     * Only genuine impossibilities are rejected here: no running server, and no
     * readable state. Neither has anything useful to say beyond itself.
     */
    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ServerAlive()) {
            RCLCPP_ERROR(get_logger(), "rejecting: %s",
                aico2_rt::RejectReasonName(aico2_rt::RejectReason::kServerNotRunning));
            return rclcpp_action::GoalResponse::REJECT;
        }
        (void)goal;  // read in Execute via handle->get_goal()
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /** @return an empty string when the goal is usable, else why not. */
    std::string Validate(const trajectory_msgs::msg::JointTrajectory& jt,
        std::vector<aico2_rt::Point>& pts, const aico2_rt::Measured& meas)
    {
        const auto convert_why = Convert(jt, pts);
        if (convert_why != aico2_rt::RejectReason::kNone) {
            return aico2_rt::RejectReasonName(convert_why);
        }
        const auto why = aico2_rt::ValidateTrajectory(
            pts.data(), static_cast<std::uint32_t>(pts.size()), meas.q, Config());
        if (why.ok()) {
            return {};
        }
        // Name the joint and the numbers: "outside joint limits" across nine
        // joints and hundreds of points is not something a caller can act on.
        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "%s -- joint %u (%s), point %u: asked for %.6f, bound %.6f, difference %.3g,"
            " measured %.6f",
            aico2_rt::RejectReasonName(why.reason), why.joint,
            why.joint < joint_names_.size() ? joint_names_[why.joint].c_str() : "?", why.point,
            why.value, why.bound, why.value - why.bound,
            why.joint < aico2_rt::kMaxDof ? meas.q[why.joint] : 0.0);
        return buf;
    }

    void Execute(const std::shared_ptr<GoalHandle> handle)
    {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        std::vector<aico2_rt::Point> pts;
        std::string why;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            aico2_rt::Measured meas{};
            if (shm_->servo_enable.load(std::memory_order_acquire) != 0u) {
                // Both channels drive the same setpoint. The executor refuses
                // the goal too, but it refuses by carrying on servoing, which
                // from here is indistinguishable from a goal that vanished.
                why = aico2_rt::RejectReasonName(aico2_rt::RejectReason::kServoActive);
            } else if (!aico2_rt::ReadMeasured(*shm_, meas)) {
                why = "no coherent state from the RT server";
            } else if (meas.operational == 0u || meas.fault != 0u) {
                why = aico2_rt::RejectReasonName(aico2_rt::RejectReason::kNotOperational);
            } else {
                why = Validate(handle->get_goal()->trajectory, pts, meas);
            }
        }
        if (!why.empty()) {
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            result->error_string = why;
            RCLCPP_ERROR(get_logger(), "goal refused: %s", why.c_str());
            handle->abort(result);
            return;
        }
        RCLCPP_INFO(get_logger(), "executing: %zu points, %.2f s", pts.size(), pts.back().t);

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
                if (state == aico2_rt::ExecState::kServoing) {
                    // Servo mode was enabled behind this goal's back. The
                    // stream has the arm now, and waiting out the full timeout
                    // would tell the caller nothing.
                    result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                    result->error_string = "pre-empted: servo mode was enabled";
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

        aico2_rt::GripperState gs{};
        if (aico2_rt::ReadGripperState(*shm_, gs) && gs.ready != 0u) {
            sensor_msgs::msg::JointState gmsg;
            gmsg.header.stamp = now();
            gmsg.name = {"gripper_width"};
            gmsg.position = {gs.width};
            gmsg.velocity = {};
            gmsg.effort = {gs.force};
            gripper_state_pub_->publish(gmsg);
        }
    }

    std::string shm_name_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> servo_joint_names_;
    std::uint32_t servo_mask_ = 0u;
    double start_tolerance_ = 0.05;
    double rest_tolerance_ = 0.01;
    int shm_fd_ = -1;
    aico2_rt::Shm* shm_ = nullptr;
    std::uint64_t goal_counter_ = 0;
    std::mutex mutex_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::CallbackGroup::SharedPtr servo_cb_group_;
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
    rclcpp::CallbackGroup::SharedPtr js_cb_group_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_state_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_srv_;
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
