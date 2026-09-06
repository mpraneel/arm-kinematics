#pragma once

#include <optional>
#include <vector>

#include "arm/kinematics.hpp"

namespace arm {

/// Every closed form solution for a target, with enough context to explain why
/// there are zero, one or two of them.
struct AnalyticIkResult {
    /// Elbow-up and elbow-down when the target is strictly inside the annulus,
    /// one at full extension or at the inner boundary, none outside it.
    std::vector<Eigen::VectorXd> solutions;
    bool reachable = false;
    /// True at the annulus boundaries, where the two branches collapse into one.
    bool at_boundary = false;

    bool empty() const { return solutions.empty(); }
    std::size_t size() const { return solutions.size(); }
};

/// Closed form 2R position IK. Law of cosines for the elbow, atan2 for the
/// shoulder. `tol` widens the annulus so that targets a hair outside it are
/// still solved, at full extension.
AnalyticIkResult analyticIk2R(const ArmModel& model, const Eigen::Vector2d& target,
                              double tol = 1e-9);

/// Closed form 3R IK for a position plus an end effector heading. The wrist
/// centre is fixed by the heading, which reduces the problem to the 2R case.
AnalyticIkResult analyticIk3R(const ArmModel& model, const Eigen::Vector2d& target,
                              double ee_orientation, double tol = 1e-9);

/// Dispatches to the 2R or 3R solver by the model's degrees of freedom.
/// `ee_orientation` is required for 3R and ignored for 2R.
AnalyticIkResult analyticIk(const ArmModel& model, const Eigen::Vector2d& target,
                            std::optional<double> ee_orientation = std::nullopt,
                            double tol = 1e-9);

/// Drops the solutions that violate a joint limit. Elbow-down may be legal
/// where elbow-up is not, so this is a filter, not a rejection.
AnalyticIkResult filterToLimits(const ArmModel& model, const AnalyticIkResult& in,
                                double tol = 1e-9);

/// The solution closest to a reference configuration in wrapped joint space,
/// or nullopt when there are none.
std::optional<Eigen::VectorXd> nearestSolution(const AnalyticIkResult& in,
                                               const Eigen::VectorXd& reference);

/// Every closed form branch for a position-only target. A 3R chain is
/// redundant for a position, so the end effector heading is swept and the
/// closed form solved at each sample; a 2R chain returns its two branches
/// directly. Used by the supervisor, which has to check all branches before
/// rejecting a target: elbow-down may be legal where elbow-up is not.
std::vector<Eigen::VectorXd> ikBranches(const ArmModel& model, const Eigen::Vector2d& target,
                                        int orientation_samples = 16, double tol = 1e-9);

struct DlsOptions {
    int max_iterations = 200;
    /// Convergence tolerance on the Cartesian error norm.
    double tolerance = 1e-8;
    /// Damping that is always applied. Zero makes the step a plain
    /// pseudoinverse, which blows up near a singularity.
    double lambda = 1e-3;
    /// Extra damping blended in as the smallest singular value drops below
    /// `sigma_threshold`. Set `adaptive` false to use `lambda` alone.
    bool adaptive = true;
    double lambda_max = 0.2;
    double sigma_threshold = 0.05;
    /// Cap on the norm of a single joint space step, in radians.
    double max_step = 0.3;
    /// Project each iterate back inside the joint limits.
    bool respect_limits = false;
    /// Also drive the end effector heading to `target_orientation`.
    bool solve_orientation = false;
    double target_orientation = 0.0;
};

struct DlsResult {
    Eigen::VectorXd q;
    bool converged = false;
    int iterations = 0;
    /// Cartesian error norm at the returned configuration.
    double error = 0.0;
    /// Smallest singular value of the Jacobian there - how close the answer
    /// sits to a singularity.
    double sigma_min = 0.0;
};

/// Damped least squares, a.k.a. Levenberg-Marquardt:
///
///     dq = J^T (J J^T + lambda^2 I)^-1 e
///
/// with lambda scaled up as the smallest singular value of J drops, so the
/// step stays bounded through a singularity instead of diverging.
DlsResult dampedLeastSquaresIk(const ArmModel& model, const Eigen::Vector2d& target,
                               const Eigen::VectorXd& q_seed, const DlsOptions& opts = {});

}  // namespace arm
