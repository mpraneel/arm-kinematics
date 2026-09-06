#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "arm/collision.hpp"
#include "arm/kinematics.hpp"

namespace arm {

/// The ways the command source is wrong on purpose. A command can carry more
/// than one, so they are flags.
enum Fault : unsigned {
    kFaultNone = 0,
    kFaultNoise = 1u << 0,
    kFaultUnreachable = 1u << 1,
    kFaultStale = 1u << 2,
    kFaultVelocitySpike = 1u << 3,
    kFaultSingular = 1u << 4,
    kFaultObstacle = 1u << 5,
};

std::string faultNames(unsigned faults);

struct Command {
    int index = 0;
    double time = 0.0;
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
    /// The command before the failure modes were applied, for reporting.
    Eigen::Vector2d nominal = Eigen::Vector2d::Zero();
    unsigned faults = kFaultNone;
};

/// Every failure mode is seeded and reproducible, which is what makes the
/// supervisor measurements repeatable rather than anecdotal.
struct ControllerConfig {
    std::uint64_t seed = 42;
    double dt = 0.02;

    /// Gaussian noise on the target position, in metres.
    double noise_stddev = 0.0;
    /// Chance per step of a target outside the reachable workspace.
    double p_unreachable = 0.0;
    /// Chance per step of holding the previous setpoint for `stale_steps`.
    double p_stale = 0.0;
    int stale_steps = 5;
    /// Chance per step of a jump large enough to break the joint rate limits.
    double p_velocity_spike = 0.0;
    /// Chance per step of a target sitting exactly at full extension.
    double p_singular = 0.0;
    /// Chance per step of a target on the far side of an obstacle, so the arm
    /// has to drive a link straight through it. Held for `obstacle_steps`,
    /// because a rate limited arm creeps rather than teleports and a
    /// single-cycle bad waypoint would never actually reach the obstacle.
    double p_obstacle = 0.0;
    int obstacle_steps = 6;
};

/// A scripted command source that follows a smooth nominal path and corrupts
/// it in specific, seeded ways.
class ScriptedController {
  public:
    ScriptedController(ArmModel model, World world, ControllerConfig config);

    Command next();
    void reset();

    const ControllerConfig& config() const { return config_; }
    /// The uncorrupted setpoint at a given time, useful for scoring.
    Eigen::Vector2d nominalAt(double t) const;

  private:
    ArmModel model_;
    World world_;
    ControllerConfig config_;

    std::uint64_t rng_state_ = 0;
    int index_ = 0;
    double time_ = 0.0;
    int stale_remaining_ = 0;
    int obstacle_remaining_ = 0;
    Eigen::Vector2d obstacle_target_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_emitted_ = Eigen::Vector2d::Zero();
    bool has_last_ = false;

    double uniform();
    double gaussian();
};

}  // namespace arm
