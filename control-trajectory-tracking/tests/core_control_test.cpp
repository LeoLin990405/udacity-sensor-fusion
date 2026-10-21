#include "control_errors.h"
#include "pid_controller.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void ExpectNear(double actual, double expected, double tolerance,
                const std::string& label) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << label << " expected " << expected
                  << ", got " << actual << "\n";
        ++failures;
    }
}

void ExpectTrue(bool condition, const std::string& label) {
    if (!condition) {
        std::cerr << "FAIL: " << label << "\n";
        ++failures;
    }
}

void TestPidTermsAndLimits() {
    PID proportional;
    proportional.Init(2.0, 0.0, 0.0, 10.0, -10.0);
    proportional.UpdateError(1.5);
    ExpectNear(proportional.TotalError(), 3.0, 1e-12,
               "proportional term");

    PID integral;
    integral.Init(0.0, 1.0, 0.0, 10.0, -10.0);
    integral.UpdateDeltaTime(0.2);
    integral.UpdateError(2.0);
    integral.UpdateError(2.0);
    ExpectNear(integral.TotalError(), 0.8, 1e-12,
               "discrete integral term");

    PID derivative;
    derivative.Init(0.0, 0.0, 1.0, 10.0, -10.0);
    derivative.UpdateDeltaTime(0.1);
    derivative.UpdateError(1.0);
    ExpectNear(derivative.TotalError(), 0.0, 1e-12,
               "first derivative sample");
    derivative.UpdateError(1.2);
    ExpectNear(derivative.TotalError(), 2.0, 1e-12,
               "discrete derivative term");

    PID limited;
    limited.Init(10.0, 0.0, 0.0, 1.2, -1.2);
    limited.UpdateError(10.0);
    ExpectNear(limited.TotalError(), 1.2, 1e-12,
               "upper output clamp");
    limited.UpdateError(-10.0);
    ExpectNear(limited.TotalError(), -1.2, 1e-12,
               "lower output clamp");

    PID safe_time;
    safe_time.Init(0.0, 0.0, 1.0, 100.0, -100.0);
    ExpectNear(safe_time.UpdateDeltaTime(0.0), 0.05, 1e-12,
               "zero delta time fallback");
    ExpectNear(safe_time.UpdateDeltaTime(10.0), 1.0, 1e-12,
               "large delta time clamp");
}

void TestTrackingErrors() {
    const std::vector<double> x{0.0, 5.0, 10.0};
    const std::vector<double> y{0.0, 5.0, 10.0};
    const std::vector<double> velocity{2.0, 4.0, 8.0};

    ExpectNear(control::ComputeThrottleError(5.5, velocity), 2.5, 1e-12,
               "desired speed minus actual speed");
    ExpectNear(control::ComputeSteeringError(0.0, 0.0, 0.0, x, y),
               std::acos(-1.0) / 4.0, 1e-12,
               "planned heading minus current yaw");

    const std::vector<double> wrap_x{-10.0};
    const std::vector<double> wrap_y{-0.1};
    const double wrapped = control::ComputeSteeringError(
        0.0, 0.0, 3.13, wrap_x, wrap_y);
    ExpectTrue(std::abs(wrapped) < 0.1, "steering error wraps at pi");
    ExpectNear(control::NormalizeAngle(3.0 * std::acos(-1.0)),
               std::acos(-1.0), 1e-12, "angle normalization");
}

void TestClosedLoopDirection() {
    PID throttle;
    throttle.Init(0.20, 0.0009, 0.10, 1.0, -1.0);
    throttle.UpdateDeltaTime(0.05);

    double speed = 0.0;
    const std::vector<double> target_speed{8.0};
    for (int step = 0; step < 500; ++step) {
        const double error =
            control::ComputeThrottleError(speed, target_speed);
        throttle.UpdateError(error);
        speed += throttle.TotalError() * 0.05 * 2.0;
    }
    ExpectTrue(speed > 7.5 && speed < 8.5,
               "positive throttle error converges toward target speed");

    PID steering;
    steering.Init(0.30, 0.001, 0.30, 1.2, -1.2);
    steering.UpdateDeltaTime(0.05);
    double yaw = 0.0;
    const std::vector<double> target_x{10.0};
    const std::vector<double> target_y{10.0};
    for (int step = 0; step < 600; ++step) {
        const double error = control::ComputeSteeringError(
            0.0, 0.0, yaw, target_x, target_y);
        steering.UpdateError(error);
        yaw = control::NormalizeAngle(
            yaw + steering.TotalError() * 0.05 * 0.8);
    }
    ExpectNear(yaw, std::acos(-1.0) / 4.0, 0.03,
               "positive steering error turns toward planned heading");
}

}  // namespace

int main() {
    TestPidTermsAndLimits();
    TestTrackingErrors();
    TestClosedLoopDirection();

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All PID and tracking-error tests passed\n";
    return EXIT_SUCCESS;
}
