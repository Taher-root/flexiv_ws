/**
 * @file joint_map_probe.cpp
 * @brief Which indices of the 9-element vector are the waist? Read-only.
 *
 * `info().DoF_e` says there are two external axes and `DoF_m` says seven arm
 * joints, but neither says where in the vector they sit. Evidence from
 * `basics1_display_robot_states` is contradictory: `temperature` and `tau_ext`
 * read zero at indices 0 and 1, which argues the external axes come first,
 * while the measured pose has plausible waist values at indices 7 and 8.
 *
 * The limits settle it without commanding anything. `RobotInfo` carries q_min,
 * q_max, dq_max and tau_max, all of length DoF, and the waist envelope measured
 * on this machine is distinctive -- AGV_Joint2 cannot go below +2.50 degrees,
 * and a positive lower limit is not something an arm joint has.
 *
 * This never enables the robot, never switches mode and never streams. It does
 * take the one RDK session, so stop the Python driver first.
 *
 *   ./joint_map_probe Rizon4-063352
 */

#include <flexiv/rdk/robot.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr double kRad2Deg = 57.29577951308232;

/** Measured waist envelope, from README.md section 2. */
constexpr double kWaistYawDeg = 87.45;    // AGV_Joint1: +/- this
constexpr double kWaistPitchMinDeg = 2.50;   // AGV_Joint2: this .. kWaistYawDeg
constexpr double kWaistPitchMaxDeg = 87.45;

bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

/** Does this joint's envelope match a waist axis as measured? */
const char* ClassifyJoint(double min_deg, double max_deg)
{
    if (Near(min_deg, kWaistPitchMinDeg, 1.0) && Near(max_deg, kWaistPitchMaxDeg, 1.5)) {
        return "WAIST pitch (AGV_Joint2): positive lower limit, unique to the waist";
    }
    if (Near(min_deg, -kWaistYawDeg, 1.5) && Near(max_deg, kWaistYawDeg, 1.5)) {
        return "WAIST yaw (AGV_Joint1): symmetric +/-87.45";
    }
    return "arm joint";
}

void PrintVec(const char* name, const std::vector<double>& v, const char* unit, double scale)
{
    std::printf("%-10s", name);
    for (double x : v) {
        std::printf(" %9.3f", x * scale);
    }
    std::printf("   [%s]\n", unit);
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::printf("usage: %s <robot_sn>\n", argv[0]);
        return 1;
    }
    std::cout.setf(std::ios::unitbuf);

    try {
        flexiv::rdk::Robot robot(argv[1]);
        const auto info = robot.info();

        std::printf("\n%s  %s  software %s  licence %s\n", info.model_name.c_str(),
            info.serial_num.c_str(), info.software_ver.c_str(), info.license_type.c_str());
        std::printf("DoF %zu  (DoF_m arm %zu, DoF_e external %zu)\n\n", info.DoF, info.DoF_m,
            info.DoF_e);

        std::printf("index     ");
        for (std::size_t i = 0; i < info.DoF; ++i) {
            std::printf(" %9zu", i);
        }
        std::printf("\n");
        PrintVec("q_min", info.q_min, "deg", kRad2Deg);
        PrintVec("q_max", info.q_max, "deg", kRad2Deg);
        PrintVec("dq_max", info.dq_max, "deg/s", kRad2Deg);
        PrintVec("tau_max", info.tau_max, "Nm", 1.0);
        PrintVec("K_q_nom", info.K_q_nom, "Nm/rad", 1.0);

        const auto states = robot.states();
        PrintVec("q now", states.q, "deg", kRad2Deg);
        PrintVec("temp", states.temperature, "C", 1.0);

        std::printf("\n--- classification, from the measured waist envelope ---\n");
        std::size_t waist_found = 0;
        for (std::size_t i = 0; i < info.DoF && i < info.q_min.size(); ++i) {
            const double lo = info.q_min[i] * kRad2Deg;
            const double hi = info.q_max[i] * kRad2Deg;
            const char* what = ClassifyJoint(lo, hi);
            const bool is_waist = (what[0] == 'W');
            waist_found += is_waist ? 1 : 0;
            std::printf("  [%zu] %8.2f .. %8.2f deg   %s\n", i, lo, hi, what);
        }

        std::printf("\n");
        if (waist_found == info.DoF_e) {
            std::printf("Found exactly %zu waist axes, matching DoF_e. Use these indices for\n"
                        "the joint map in aico2_rt_server.\n", waist_found);
        } else {
            std::printf("Found %zu candidate waist axes but DoF_e is %zu -- the envelope match is\n"
                        "inconclusive. Do NOT guess: command one waist axis a few degrees with\n"
                        "scripts/rdk_waist_move.py and see which index changes.\n",
                waist_found, info.DoF_e);
        }
        return 0;

    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
