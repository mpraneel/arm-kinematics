#include "arm/controller.hpp"

#include <cmath>

namespace arm {
namespace {

/// splitmix64: small, fast, and identical on every platform, so a seed means
/// the same command sequence everywhere.
std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

}  // namespace

std::string faultNames(unsigned faults) {
    if (faults == kFaultNone) return "none";
    std::string out;
    const auto add = [&](unsigned bit, const char* name) {
        if (faults & bit) {
            if (!out.empty()) out += "|";
            out += name;
        }
    };
    add(kFaultNoise, "noise");
    add(kFaultUnreachable, "unreachable");
    add(kFaultStale, "stale");
    add(kFaultVelocitySpike, "velocity_spike");
    add(kFaultSingular, "singular");
    add(kFaultObstacle, "obstacle");
    return out;
}

ScriptedController::ScriptedController(ArmModel model, World world, ControllerConfig config)
    : model_(std::move(model)), world_(std::move(world)), config_(config) {
    reset();
}

void ScriptedController::reset() {
    rng_state_ = config_.seed;
    index_ = 0;
    time_ = 0.0;
    stale_remaining_ = 0;
    obstacle_remaining_ = 0;
    obstacle_target_.setZero();
    has_last_ = false;
    last_emitted_.setZero();
}

double ScriptedController::uniform() {
    // 53 bits of mantissa, mapped to [0, 1).
    return (splitmix64(rng_state_) >> 11) * (1.0 / 9007199254740992.0);
}

double ScriptedController::gaussian() {
    const double u1 = std::max(uniform(), 1e-12);
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

Eigen::Vector2d ScriptedController::nominalAt(double t) const {
    // A smooth sweep that stays comfortably inside the annulus, so anything
    // that goes wrong below is the injected fault and not the path.
    const double reach = model_.reach();
    const double radius = 0.45 * reach + 0.18 * reach * std::sin(0.7 * t);
    const double heading = 0.9 * std::sin(0.5 * t) + 0.25 * std::sin(1.3 * t);
    return model_.base.translation() + radius * Eigen::Vector2d(std::cos(heading), std::sin(heading));
}

Command ScriptedController::next() {
    Command cmd;
    cmd.index = index_++;
    cmd.time = time_;
    time_ += config_.dt;
    cmd.nominal = nominalAt(cmd.time);
    cmd.target = cmd.nominal;

    // A stale setpoint holds the previous command for k steps: the arm is
    // told nothing has changed when it has.
    if (stale_remaining_ > 0 && has_last_) {
        --stale_remaining_;
        cmd.target = last_emitted_;
        cmd.faults |= kFaultStale;
        return cmd;
    }
    if (config_.p_stale > 0.0 && has_last_ && uniform() < config_.p_stale) {
        stale_remaining_ = config_.stale_steps - 1;
        cmd.target = last_emitted_;
        cmd.faults |= kFaultStale;
        return cmd;
    }

    if (config_.noise_stddev > 0.0) {
        cmd.target += config_.noise_stddev * Eigen::Vector2d(gaussian(), gaussian());
        cmd.faults |= kFaultNoise;
    }

    const Eigen::Vector2d base = model_.base.translation();
    const Eigen::Vector2d dir = (cmd.target - base).normalized();

    if (config_.p_unreachable > 0.0 && uniform() < config_.p_unreachable) {
        cmd.target = base + dir * (model_.reach() * 1.25);
        cmd.faults |= kFaultUnreachable;
    }

    if (config_.p_singular > 0.0 && uniform() < config_.p_singular) {
        // Exactly at full extension: the Jacobian is rank deficient there.
        cmd.target = base + dir * model_.reach();
        cmd.faults |= kFaultSingular;
    }

    if (config_.p_velocity_spike > 0.0 && uniform() < config_.p_velocity_spike) {
        // Reflected through the base, so tracking it in one step needs a joint
        // rate no real actuator has.
        cmd.target = base - (cmd.target - base);
        cmd.faults |= kFaultVelocitySpike;
    }

    if (obstacle_remaining_ > 0) {
        --obstacle_remaining_;
        cmd.target = obstacle_target_;
        cmd.faults |= kFaultObstacle;
    } else if (config_.p_obstacle > 0.0 && !world_.empty() &&
               uniform() < config_.p_obstacle) {
        const std::size_t total = world_.circles.size() + world_.boxes.size();
        const std::size_t pick = static_cast<std::size_t>(uniform() * total) % total;

        Eigen::Vector2d centre;
        double extent = 0.0;
        if (pick < world_.circles.size()) {
            centre = world_.circles[pick].center;
            extent = world_.circles[pick].radius;
        } else {
            const Aabb& b = world_.boxes[pick - world_.circles.size()];
            centre = 0.5 * (b.min + b.max);
            extent = 0.5 * (b.max - b.min).norm();
        }

        // Just past the obstacle along the ray from the base: the tip itself
        // is in free space, so only a swept path check catches that a link has
        // to pass straight through the obstacle to get there.
        const Eigen::Vector2d ray = (centre - base).normalized();
        const double radius = std::min((centre - base).norm() + extent + 0.2, model_.reach());
        obstacle_target_ = base + ray * radius;
        obstacle_remaining_ = std::max(0, config_.obstacle_steps - 1);
        cmd.target = obstacle_target_;
        cmd.faults |= kFaultObstacle;
    }

    last_emitted_ = cmd.target;
    has_last_ = true;
    return cmd;
}

}  // namespace arm
