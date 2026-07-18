#include "control_errors.h"
#include "pid_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

void SimulateThrottle(const std::string& output_path,
                      const std::string& controller_name,
                      double kp, double ki, double kd, bool append) {
    std::ofstream output(output_path, append ? std::ios::app : std::ios::out);
    if (!append) {
        output << "controller,step,target,actual,error,command\n";
    }

    PID pid;
    pid.Init(kp, ki, kd, 1.0, -1.0);
    pid.UpdateDeltaTime(0.05);
    double velocity = 0.0;

    for (int step = 0; step < 800; ++step) {
        const double target = step < 400 ? 8.0 : 5.0;
        const std::vector<double> planned_velocity{target};
        const double error =
            control::ComputeThrottleError(velocity, planned_velocity);
        pid.UpdateError(error);
        const double command = pid.TotalError();

        // A deterministic first-order plant is sufficient to compare gains
        // while exercising the production controller implementation.
        const double acceleration = 2.2 * command - 0.015 * velocity;
        velocity = std::max(0.0, velocity + acceleration * 0.05);
        output << controller_name << ',' << step << ',' << target << ','
               << velocity << ',' << error << ',' << command << '\n';
    }
}

void SimulateSteering(const std::string& output_path,
                      const std::string& controller_name,
                      double kp, double ki, double kd, bool append) {
    std::ofstream output(output_path, append ? std::ios::app : std::ios::out);
    if (!append) {
        output << "controller,step,target,actual,error,command\n";
    }

    PID pid;
    pid.Init(kp, ki, kd, 1.2, -1.2);
    pid.UpdateDeltaTime(0.05);
    double yaw = 0.0;

    for (int step = 0; step < 800; ++step) {
        const double target_yaw = step < 400 ? 0.65 : -0.35;
        const std::vector<double> target_x{10.0 * std::cos(target_yaw)};
        const std::vector<double> target_y{10.0 * std::sin(target_yaw)};
        const double error = control::ComputeSteeringError(
            0.0, 0.0, yaw, target_x, target_y);
        pid.UpdateError(error);
        const double command = pid.TotalError();

        yaw = control::NormalizeAngle(yaw + 0.8 * command * 0.05);
        output << controller_name << ',' << step << ',' << target_yaw << ','
               << yaw << ',' << error << ',' << command << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: response_experiment OUTPUT_DIRECTORY\n";
        return EXIT_FAILURE;
    }

    const std::string output_directory = argv[1];
    const std::string throttle_path =
        output_directory + "/throttle_gain_sweep.csv";
    const std::string steering_path =
        output_directory + "/steering_gain_sweep.csv";

    SimulateThrottle(throttle_path, "P", 0.20, 0.0, 0.0, false);
    SimulateThrottle(throttle_path, "PI", 0.20, 0.0009, 0.0, true);
    SimulateThrottle(throttle_path, "PID", 0.20, 0.0009, 0.10, true);

    SimulateSteering(steering_path, "P", 0.30, 0.0, 0.0, false);
    SimulateSteering(steering_path, "PI", 0.30, 0.001, 0.0, true);
    SimulateSteering(steering_path, "PID", 0.30, 0.001, 0.30, true);

    std::cout << "Wrote deterministic response data to "
              << output_directory << '\n';
    return EXIT_SUCCESS;
}
