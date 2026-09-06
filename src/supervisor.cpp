#include "arm/supervisor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace arm {
namespace {

/// Point at fraction t along the path the arm actually travels.
Eigen::VectorXd lerpJoints(const ArmModel& model, const Eigen::VectorXd& a,
                           const Eigen::VectorXd& b, double t) {
    return a + t * jointDelta(model, a, b);
}

/// Motion exactly at the rate limit is legal; this absorbs the rounding that
/// a clamp to exactly that rate leaves behind.
constexpr double kRateTolerance = 1e-9;

std::string vecToJson(const Eigen::VectorXd& v) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6) << "[";
    for (Eigen::Index i = 0; i < v.size(); ++i) {
        if (i) os << ",";
        os << v[i];
    }
    os << "]";
    return os.str();
}

double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double idx = p * (values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
    const double frac = idx - lo;
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

}  // namespace

const char* checkName(Check c) {
    switch (c) {
        case Check::None: return "none";
        case Check::Reachability: return "reachability";
        case Check::JointLimits: return "joint_limits";
        case Check::VelocityLimits: return "velocity_limits";
        case Check::Singularity: return "singularity";
        case Check::Collision: return "collision";
    }
    return "unknown";
}

unsigned checkBit(Check c) { return 1u << static_cast<unsigned>(c); }

const char* fallbackName(Fallback f) {
    switch (f) {
        case Fallback::Hold: return "hold";
        case Fallback::Clamp: return "clamp";
        case Fallback::Plan: return "plan";
    }
    return "unknown";
}

std::string CommandRecord::toJson() const {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6);
    os << "{\"index\":" << index << ",\"t\":" << time << ",\"target\":[" << target.x() << ","
       << target.y() << "]"
       << ",\"faults\":\"" << faultNames(faults) << "\""
       << ",\"supervised\":" << (supervised ? "true" : "false")
       << ",\"failed_check\":\"" << checkName(failed_check) << "\""
       << ",\"triggered\":" << triggered_checks << ",\"intervened\":" << (intervened ? "true" : "false")
       << ",\"fallback\":\"" << (fallback_engaged ? fallbackName(fallback) : "none") << "\""
       << ",\"fallback_ok\":" << (fallback_ok ? "true" : "false")
       << ",\"latency_us\":" << std::setprecision(3) << latency_us << std::setprecision(6)
       << ",\"limit_violation\":" << (limit_violation ? "true" : "false")
       << ",\"velocity_violation\":" << (velocity_violation ? "true" : "false")
       << ",\"collision\":" << (collision ? "true" : "false")
       << ",\"reached\":" << (reached_target ? "true" : "false")
       << ",\"sigma_min\":" << sigma_min << ",\"q\":" << vecToJson(q_after) << "}";
    return os.str();
}

RunMetrics RunMetrics::fromRecords(const std::vector<CommandRecord>& records) {
    RunMetrics m;
    std::vector<double> latencies;
    latencies.reserve(records.size());

    for (const CommandRecord& r : records) {
        ++m.commands;
        if (r.limit_violation) ++m.joint_limit_violations;
        if (r.velocity_violation) ++m.velocity_violations;
        if (r.collision) ++m.collisions;
        if (r.reached_target) ++m.targets_reached;
        // Noise is a fact of life rather than a bad command, so a noisy but
        // otherwise sound target still counts as one the arm ought to reach.
        if ((r.faults & ~static_cast<unsigned>(kFaultNoise)) == 0) {
            ++m.clean_commands;
            if (r.reached_target) ++m.clean_reached;
        }
        if (r.intervened) ++m.interventions;
        if (r.fallback_engaged && !r.fallback_ok) ++m.fallback_failures;
        latencies.push_back(r.latency_us);
    }

    if (!latencies.empty()) {
        m.p50_latency_us = percentile(latencies, 0.50);
        m.p99_latency_us = percentile(latencies, 0.99);
        double sum = 0.0;
        for (double l : latencies) sum += l;
        m.mean_latency_us = sum / latencies.size();
    }
    return m;
}

Supervisor::Supervisor(ArmModel model, World world, SupervisorConfig config)
    : model_(std::move(model)), world_(std::move(world)), config_(std::move(config)) {
    q_ = Eigen::VectorXd::Zero(model_.link_lengths.size());
}

Eigen::VectorXd Supervisor::naiveIk(const Eigen::Vector2d& target) const {
    DlsOptions opts;
    opts.max_iterations = 100;
    opts.tolerance = 1e-6;
    opts.max_step = 0.5;
    opts.respect_limits = false;  // the whole point of the baseline
    return dampedLeastSquaresIk(model_, target, q_, opts).q;
}

std::optional<Eigen::VectorXd> Supervisor::bestLegalBranch(const Eigen::Vector2d& target) const {
    const std::vector<Eigen::VectorXd> branches =
        ikBranches(model_, target, config_.orientation_samples);
    std::optional<Eigen::VectorXd> best;
    double best_cost = 0.0;
    for (const Eigen::VectorXd& q : branches) {
        if (!model_.withinLimits(q)) continue;
        const double cost = jointDelta(model_, q_, q).norm();
        if (!best || cost < best_cost) {
            best = q;
            best_cost = cost;
        }
    }
    return best;
}

bool Supervisor::velocityWithinLimits(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    if (config_.velocity_limits.size() == 0) return true;
    const Eigen::VectorXd rate = jointDelta(model_, from, to).cwiseAbs() / config_.dt;
    for (Eigen::Index i = 0; i < rate.size(); ++i) {
        if (rate[i] > config_.velocity_limits[i] + kRateTolerance) return false;
    }
    return true;
}

Eigen::VectorXd Supervisor::clampVelocity(const Eigen::VectorXd& from,
                                          const Eigen::VectorXd& to) const {
    if (config_.velocity_limits.size() == 0) return to;
    const Eigen::VectorXd delta = jointDelta(model_, from, to);
    // Scale the whole step by the worst offending joint, so the direction of
    // the motion is preserved rather than skewed.
    double scale = 1.0;
    for (Eigen::Index i = 0; i < delta.size(); ++i) {
        const double allowed = config_.velocity_limits[i] * config_.dt;
        const double magnitude = std::abs(delta[i]);
        if (magnitude > allowed && magnitude > 1e-12) {
            scale = std::min(scale, allowed / magnitude);
        }
    }
    // A joint limited step can be scaled but must not be wrapped: wrapping is
    // what would carry a limited joint through its own end stop.
    return from + scale * delta;
}

bool Supervisor::isFeasible(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    if (!model_.withinLimits(to)) return false;
    if (!velocityWithinLimits(from, to)) return false;
    if (manipulability(model_, to).sigma_min < config_.sigma_min_threshold) return false;
    if (checkPath(model_, from, to, world_, config_.path_steps, config_.collision_margin).hit()) {
        return false;
    }
    return true;
}

Eigen::VectorXd Supervisor::furthestFeasible(const Eigen::VectorXd& from,
                                             const Eigen::VectorXd& to) const {
    constexpr int kSamples = 12;
    for (int i = kSamples; i > 0; --i) {
        const double t = static_cast<double>(i) / kSamples;
        const Eigen::VectorXd candidate = lerpJoints(model_, from, to, t);
        if (isFeasible(from, candidate)) return candidate;
    }
    return from;  // nothing along this step is legal, so hold
}

CommandRecord Supervisor::step(const Command& command) {
    CommandRecord rec;
    rec.index = command.index;
    rec.time = command.time;
    rec.target = command.target;
    rec.faults = command.faults;
    rec.supervised = config_.enabled;
    rec.fallback = config_.fallback;
    rec.q_before = q_;

    // The IK solve is work the arm does either way, so it sits outside the
    // latency measurement: what is being measured is the guardrail's own cost.
    const Eigen::VectorXd q_cmd = naiveIk(command.target);

    if (!config_.enabled) {
        q_ = q_cmd;
    } else {
        const auto t0 = std::chrono::steady_clock::now();
        Eigen::VectorXd q_goal = q_cmd;
        Check failed = Check::None;

        // 1. Reachability. Cheap, so it goes first.
        const double radius = (command.target - model_.base.translation()).norm();
        if (radius > model_.reach() || radius < model_.innerReach() ||
            world_.occupied(command.target, config_.collision_margin)) {
            failed = Check::Reachability;
            rec.triggered_checks |= checkBit(Check::Reachability);
        }

        // 2. Joint limits, across every closed form branch. Elbow-down may be
        // legal where elbow-up is not, so a branch switch is a modification
        // rather than a rejection.
        if (failed == Check::None && !model_.withinLimits(q_goal)) {
            if (const std::optional<Eigen::VectorXd> branch = bestLegalBranch(command.target)) {
                q_goal = *branch;
                rec.intervened = true;
            } else {
                failed = Check::JointLimits;
            }
            rec.triggered_checks |= checkBit(Check::JointLimits);
        }

        // The configuration that would actually realize the commanded target,
        // before any rate limiting. The singularity guard judges this one: what
        // matters is whether the target is near singular, not whether this
        // cycle's scaled down step happens to stop short of it.
        const Eigen::VectorXd q_commanded = q_goal;

        // 3. Velocity limits: clamp or reject, by policy.
        if (failed == Check::None && !velocityWithinLimits(q_, q_goal)) {
            if (config_.clamp_velocity) {
                q_goal = clampVelocity(q_, q_goal);
                rec.intervened = true;
            } else {
                failed = Check::VelocityLimits;
            }
            rec.triggered_checks |= checkBit(Check::VelocityLimits);
        }

        // 4. Singularity guard. Refusing a near singular target is more useful
        // than trying to recover from one.
        if (failed == Check::None &&
            manipulability(model_, q_commanded).sigma_min < config_.sigma_min_threshold) {
            failed = Check::Singularity;
            rec.triggered_checks |= checkBit(Check::Singularity);
        }

        // 5. Collision, over the swept path and not just the endpoint.
        if (failed == Check::None &&
            checkPath(model_, q_, q_goal, world_, config_.path_steps, config_.collision_margin)
                .hit()) {
            failed = Check::Collision;
            rec.triggered_checks |= checkBit(Check::Collision);
        }

        if (failed != Check::None) {
            rec.failed_check = failed;
            rec.intervened = true;
            rec.fallback_engaged = true;

            switch (config_.fallback) {
                case Fallback::Hold:
                    q_goal = q_;
                    rec.fallback_ok = true;
                    break;
                case Fallback::Clamp: {
                    q_goal = furthestFeasible(q_, q_cmd);
                    rec.fallback_ok = jointDelta(model_, q_, q_goal).norm() > 1e-9;
                    break;
                }
                case Fallback::Plan: {
                    // Planning only makes sense towards a goal that is itself
                    // legal, so an illegal goal degrades to clamping.
                    std::optional<Eigen::VectorXd> goal;
                    if (model_.withinLimits(q_cmd) &&
                        !checkCollision(model_, q_cmd, world_, config_.collision_margin).hit()) {
                        goal = q_cmd;
                    } else if (const auto branch = bestLegalBranch(command.target)) {
                        if (!checkCollision(model_, *branch, world_, config_.collision_margin)
                                 .hit()) {
                            goal = *branch;
                        }
                    }

                    if (goal) {
                        const RrtResult plan = planRrt(model_, q_, *goal, world_, config_.rrt);
                        if (plan.success && plan.path.size() > 1) {
                            // Execute one control step along the plan.
                            Eigen::VectorXd stepped = clampVelocity(q_, plan.path[1]);
                            if (!isFeasible(q_, stepped)) {
                                stepped = furthestFeasible(q_, plan.path[1]);
                            }
                            q_goal = stepped;
                            rec.fallback_ok = jointDelta(model_, q_, q_goal).norm() > 1e-9;
                            break;
                        }
                    }
                    q_goal = furthestFeasible(q_, q_cmd);
                    rec.fallback_ok = jointDelta(model_, q_, q_goal).norm() > 1e-9;
                    break;
                }
            }
        }

        q_ = q_goal;
        rec.latency_us = std::chrono::duration<double, std::micro>(
                             std::chrono::steady_clock::now() - t0)
                             .count();
    }

    rec.q_after = q_;
    // Ground truth, measured on what the arm actually did rather than on what
    // the supervisor believed.
    rec.limit_violation = !model_.withinLimits(q_);
    rec.velocity_violation = !velocityWithinLimits(rec.q_before, q_);
    rec.collision =
        checkPath(model_, rec.q_before, q_, world_, config_.path_steps, 0.0).hit();
    rec.reached_target =
        (eePosition(model_, q_) - command.target).norm() <= config_.reach_tolerance;
    rec.sigma_min = manipulability(model_, q_).sigma_min;
    return rec;
}

}  // namespace arm
