#include "arm/scenario.hpp"

#include "arm/ik.hpp"

namespace arm::scenario {

ArmModel model() {
    Eigen::VectorXd lengths(3);
    lengths << 1.0, 0.8, 0.6;
    ArmModel m(lengths);
    m.joint_min << -3.1, -2.5, -3.1;
    m.joint_max << 3.1, 2.5, 3.1;
    m.link_radius = 0.04;
    return m;
}

World world() {
    World w;
    w.circles.push_back(Circle{Eigen::Vector2d(-1.25, 0.55), 0.25});
    w.circles.push_back(Circle{Eigen::Vector2d(0.3, 1.7), 0.22});
    w.boxes.push_back(Aabb{Eigen::Vector2d(-1.7, -1.25), Eigen::Vector2d(-0.9, -0.6)});
    return w;
}

ControllerConfig controller(std::uint64_t seed, double dt) {
    ControllerConfig c;
    c.seed = seed;
    c.dt = dt;
    c.noise_stddev = 0.02;
    c.p_unreachable = 0.04;
    c.p_stale = 0.05;
    c.stale_steps = 5;
    c.p_velocity_spike = 0.03;
    c.p_singular = 0.04;
    c.p_obstacle = 0.04;
    return c;
}

SupervisorConfig supervisor(bool enabled, Fallback fallback, std::uint64_t seed, double dt) {
    SupervisorConfig s;
    s.enabled = enabled;
    s.dt = dt;
    s.velocity_limits = Eigen::VectorXd::Constant(3, 3.0);
    s.sigma_min_threshold = 0.08;
    s.collision_margin = 0.005;
    s.fallback = fallback;
    s.rrt.max_iterations = 800;
    s.rrt.seed = seed;
    return s;
}

Eigen::VectorXd startConfiguration(const ArmModel& m, const Eigen::Vector2d& first_target) {
    Eigen::VectorXd seed(m.link_lengths.size());
    seed << 0.6, -1.0, 0.7;
    DlsOptions opts;
    opts.max_iterations = 200;
    opts.respect_limits = true;
    return dampedLeastSquaresIk(m, first_target, seed, opts).q;
}

}  // namespace arm::scenario
