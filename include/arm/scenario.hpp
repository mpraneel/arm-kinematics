#pragma once

#include "arm/controller.hpp"
#include "arm/supervisor.hpp"

namespace arm::scenario {

/// The scene the benchmark measures and the visualizer plays back. It lives in
/// the library so the table, the tests and the recording all describe the same
/// arm in the same world rather than three lookalikes.

/// 3R chain whose elbow cannot fold all the way back. That limit is also what
/// keeps the arm from folding onto itself, and naive IK respects neither.
ArmModel model();

/// Obstacles placed clear of the sweep the nominal path traces, so a collision
/// is always attributable to an injected fault rather than to the scene.
World world();

/// The unreliable command source, with every failure mode turned on.
ControllerConfig controller(std::uint64_t seed = 42, double dt = 0.02);

SupervisorConfig supervisor(bool enabled, Fallback fallback, std::uint64_t seed = 42,
                            double dt = 0.02);

/// Start the arm already on the controller's nominal path, so a policy that
/// refuses large steps is not judged on an opening jump no real system would
/// have issued.
Eigen::VectorXd startConfiguration(const ArmModel& model, const Eigen::Vector2d& first_target);

}  // namespace arm::scenario
