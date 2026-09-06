#pragma once

#include <string>
#include <vector>

#include "arm/collision.hpp"
#include "arm/controller.hpp"
#include "arm/ik.hpp"
#include "arm/kinematics.hpp"
#include "arm/rrt.hpp"

namespace arm {

/// The checks, in the order they run: cheapest and most decisive first.
enum class Check { None, Reachability, JointLimits, VelocityLimits, Singularity, Collision };
const char* checkName(Check c);
/// Bit for `CommandRecord::triggered_checks`.
unsigned checkBit(Check c);

/// What to do with a command that fails a check.
enum class Fallback {
    Hold,   ///< stay where we are
    Clamp,  ///< move as far along the commanded step as is still legal
    Plan,   ///< plan around it with an RRT in joint space
};
const char* fallbackName(Fallback f);

struct SupervisorConfig {
    /// False runs the arm with no guardrail at all, which is the baseline the
    /// measurement table compares against.
    bool enabled = true;
    double dt = 0.02;

    /// Per joint rate limits, rad/s. Empty means no limit.
    Eigen::VectorXd velocity_limits;
    /// A step that breaks the rate limits is scaled down rather than refused.
    /// Turning this off makes the velocity check a rejection like the others.
    bool clamp_velocity = true;
    /// Reject targets whose configuration is closer to singular than this.
    double sigma_min_threshold = 0.08;
    double collision_margin = 0.005;
    /// Waypoints used for the swept path collision check.
    int path_steps = 12;
    /// Branch enumeration density for the joint limit check.
    int orientation_samples = 16;
    /// How close the end effector has to get before a target counts as reached.
    double reach_tolerance = 0.02;

    Fallback fallback = Fallback::Clamp;
    RrtConfig rrt;
};

/// One line of the structured log, one per command.
struct CommandRecord {
    int index = 0;
    double time = 0.0;
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
    /// Which failure modes the controller injected into this command.
    unsigned faults = kFaultNone;
    bool supervised = false;

    Check failed_check = Check::None;
    /// Every check that fired, whether it modified the command or rejected it.
    /// A check can modify without rejecting: a branch switch keeps the target
    /// but changes how the arm gets there.
    unsigned triggered_checks = 0;
    bool intervened = false;  ///< the command was not executed as issued
    Fallback fallback = Fallback::Hold;
    bool fallback_engaged = false;
    /// False when even the fallback could not produce a legal motion.
    bool fallback_ok = true;

    Eigen::VectorXd q_before;
    Eigen::VectorXd q_after;

    /// Time spent inside the supervisor itself, excluding the IK solve the arm
    /// would have run anyway.
    double latency_us = 0.0;

    // What actually happened once the arm executed, checked independently of
    // the supervisor's own reasoning.
    bool limit_violation = false;
    bool velocity_violation = false;
    bool collision = false;
    bool reached_target = false;
    double sigma_min = 0.0;

    std::string toJson() const;
};

/// Aggregate of a run, the numbers that go in the table.
struct RunMetrics {
    int commands = 0;
    int joint_limit_violations = 0;
    int velocity_violations = 0;
    int collisions = 0;
    int targets_reached = 0;
    /// Commands carrying no fault worse than sensor noise, and how many of
    /// those the arm actually reached.
    int clean_commands = 0;
    int clean_reached = 0;
    int interventions = 0;
    int fallback_failures = 0;

    double p50_latency_us = 0.0;
    double p99_latency_us = 0.0;
    double mean_latency_us = 0.0;

    double interventionRate() const {
        return commands ? static_cast<double>(interventions) / commands : 0.0;
    }

    static RunMetrics fromRecords(const std::vector<CommandRecord>& records);
};

/// Sits between the controller and the arm. Every command passes through it,
/// and it either forwards, modifies or rejects.
class Supervisor {
  public:
    Supervisor(ArmModel model, World world, SupervisorConfig config);

    /// Run one command and move the arm. Returns the log record.
    CommandRecord step(const Command& command);

    void setConfiguration(const Eigen::VectorXd& q) { q_ = q; }
    const Eigen::VectorXd& configuration() const { return q_; }
    const SupervisorConfig& config() const { return config_; }

  private:
    /// What the arm would do with no supervisor: damped least squares from
    /// where it currently is, limits and obstacles ignored.
    Eigen::VectorXd naiveIk(const Eigen::Vector2d& target) const;
    /// The closed form branch nearest the current configuration that respects
    /// the joint limits, if there is one.
    std::optional<Eigen::VectorXd> bestLegalBranch(const Eigen::Vector2d& target) const;

    bool velocityWithinLimits(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    /// The step scaled down so no joint exceeds its rate limit.
    Eigen::VectorXd clampVelocity(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    bool isFeasible(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    /// The furthest point along from->to that is still feasible.
    Eigen::VectorXd furthestFeasible(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;

    ArmModel model_;
    World world_;
    SupervisorConfig config_;
    Eigen::VectorXd q_;
};

}  // namespace arm
