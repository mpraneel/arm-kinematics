#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <vector>

namespace arm {

/// A planar N-link revolute chain. Configuration is a vector of joint angles,
/// each measured relative to the previous link.
struct ArmModel {
    Eigen::VectorXd link_lengths;
    Eigen::VectorXd joint_min;
    Eigen::VectorXd joint_max;
    /// Capsule radius used when the links are treated as swept volumes.
    double link_radius = 0.0;
    /// World pose of the base frame (joint 1 sits at its origin).
    Eigen::Isometry2d base = Eigen::Isometry2d::Identity();

    ArmModel() = default;

    /// Uniform limits of [-pi, pi] on every joint.
    explicit ArmModel(const Eigen::VectorXd& lengths, double limit = M_PI);

    std::size_t dof() const { return static_cast<std::size_t>(link_lengths.size()); }
    double reach() const { return link_lengths.sum(); }
    /// Radius of the inner hole of the reachable annulus.
    double innerReach() const;

    bool withinLimits(const Eigen::VectorXd& q, double tol = 0.0) const;
    /// Index of the first violated joint limit, or -1 if the configuration is legal.
    int firstLimitViolation(const Eigen::VectorXd& q, double tol = 0.0) const;
    Eigen::VectorXd clampToLimits(const Eigen::VectorXd& q) const;
};

/// Every frame along the chain, not just the end effector: the intermediate
/// ones are what the renderer and the Jacobian need.
struct FkResult {
    /// frames[0] is the base; frames[i] is the frame at the tip of link i, so
    /// frames.back() is the end effector and frames[i-1] is where joint i sits.
    std::vector<Eigen::Isometry2d> frames;

    const Eigen::Isometry2d& eePose() const { return frames.back(); }
    Eigen::Vector2d eePosition() const { return frames.back().translation(); }
    /// Sum of the joint angles, i.e. the end effector heading.
    double eeOrientation() const;
    Eigen::Vector2d jointPosition(std::size_t i) const { return frames[i].translation(); }
    /// Base, then one point per link tip. Size dof() + 1.
    std::vector<Eigen::Vector2d> points() const;
};

/// Rotation by theta followed by a translation of length along the new x axis.
Eigen::Isometry2d linkTransform(double theta, double length);

FkResult forwardKinematics(const ArmModel& model, const Eigen::VectorXd& q);

/// Convenience wrapper when only the tip is wanted.
Eigen::Vector2d eePosition(const ArmModel& model, const Eigen::VectorXd& q);

/// Geometric Jacobian, 3 x n: rows are dx, dy, dtheta. For a planar revolute
/// chain this is analytic - column i is the joint axis crossed with the vector
/// from joint i to the end effector, which in 2D collapses to
/// [-(y_ee - y_i), (x_ee - x_i), 1].
Eigen::MatrixXd jacobian(const ArmModel& model, const Eigen::VectorXd& q);
Eigen::MatrixXd jacobian(const FkResult& fk);

/// The top two rows of the full Jacobian: end effector translation only.
Eigen::MatrixXd positionJacobian(const ArmModel& model, const Eigen::VectorXd& q);
Eigen::MatrixXd positionJacobian(const FkResult& fk);

/// Singular value decomposition of a Jacobian, reported in the terms that
/// matter near a singularity.
struct Manipulability {
    Eigen::VectorXd singular_values;   ///< descending
    /// Columns are the ellipsoid axis directions (left singular vectors),
    /// already scaled by their singular values.
    Eigen::MatrixXd ellipsoid_axes;
    double yoshikawa = 0.0;            ///< sqrt(det(J J^T))
    double condition_number = 0.0;     ///< sigma_max / sigma_min, inf when rank deficient
    double sigma_min = 0.0;
    double sigma_max = 0.0;
};

Manipulability manipulability(const Eigen::MatrixXd& J);
inline Manipulability manipulability(const ArmModel& model, const Eigen::VectorXd& q) {
    return manipulability(positionJacobian(model, q));
}

/// Wrap to (-pi, pi].
double wrapAngle(double a);
Eigen::VectorXd wrapAngles(const Eigen::VectorXd& q);
/// Shortest signed angular difference a - b, wrapped.
double angleDiff(double a, double b);
Eigen::VectorXd angleDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

}  // namespace arm
