#include "arm/kinematics.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace arm {

ArmModel::ArmModel(const Eigen::VectorXd& lengths, double limit)
    : link_lengths(lengths),
      joint_min(Eigen::VectorXd::Constant(lengths.size(), -limit)),
      joint_max(Eigen::VectorXd::Constant(lengths.size(), limit)) {}

double ArmModel::innerReach() const {
    if (link_lengths.size() == 0) return 0.0;
    const double longest = link_lengths.maxCoeff();
    return std::max(0.0, 2.0 * longest - link_lengths.sum());
}

int ArmModel::firstLimitViolation(const Eigen::VectorXd& q, double tol) const {
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        if (q[i] < joint_min[i] - tol || q[i] > joint_max[i] + tol) return static_cast<int>(i);
    }
    return -1;
}

bool ArmModel::withinLimits(const Eigen::VectorXd& q, double tol) const {
    return firstLimitViolation(q, tol) < 0;
}

Eigen::VectorXd ArmModel::clampToLimits(const Eigen::VectorXd& q) const {
    Eigen::VectorXd out = q;
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        out[i] = std::min(std::max(out[i], joint_min[i]), joint_max[i]);
    }
    return out;
}

double FkResult::eeOrientation() const {
    return Eigen::Rotation2Dd(frames.back().linear()).angle();
}

std::vector<Eigen::Vector2d> FkResult::points() const {
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(frames.size());
    for (const auto& f : frames) pts.push_back(f.translation());
    return pts;
}

Eigen::Isometry2d linkTransform(double theta, double length) {
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    t.linear() = Eigen::Rotation2Dd(theta).toRotationMatrix();
    t.translation() = t.linear() * Eigen::Vector2d(length, 0.0);
    return t;
}

FkResult forwardKinematics(const ArmModel& model, const Eigen::VectorXd& q) {
    if (q.size() != model.link_lengths.size()) {
        throw std::invalid_argument("forwardKinematics: configuration size does not match the model");
    }
    FkResult fk;
    fk.frames.reserve(model.dof() + 1);
    Eigen::Isometry2d acc = model.base;
    fk.frames.push_back(acc);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        acc = acc * linkTransform(q[i], model.link_lengths[i]);
        fk.frames.push_back(acc);
    }
    return fk;
}

Eigen::Vector2d eePosition(const ArmModel& model, const Eigen::VectorXd& q) {
    return forwardKinematics(model, q).eePosition();
}

Eigen::MatrixXd jacobian(const FkResult& fk) {
    const Eigen::Index n = static_cast<Eigen::Index>(fk.frames.size()) - 1;
    Eigen::MatrixXd J(3, n);
    const Eigen::Vector2d ee = fk.frames.back().translation();
    for (Eigen::Index i = 0; i < n; ++i) {
        // Joint i+1 sits at the tip of link i, i.e. at frame i.
        const Eigen::Vector2d p = fk.frames[static_cast<std::size_t>(i)].translation();
        J(0, i) = -(ee.y() - p.y());
        J(1, i) = ee.x() - p.x();
        J(2, i) = 1.0;
    }
    return J;
}

Eigen::MatrixXd jacobian(const ArmModel& model, const Eigen::VectorXd& q) {
    return jacobian(forwardKinematics(model, q));
}

Eigen::MatrixXd positionJacobian(const FkResult& fk) {
    return jacobian(fk).topRows(2);
}

Eigen::MatrixXd positionJacobian(const ArmModel& model, const Eigen::VectorXd& q) {
    return positionJacobian(forwardKinematics(model, q));
}

Manipulability manipulability(const Eigen::MatrixXd& J) {
    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(J);
    Manipulability m;
    m.singular_values = svd.singularValues();
    m.ellipsoid_axes = svd.matrixU() * m.singular_values.asDiagonal();
    // For J with full row rank, sqrt(det(J J^T)) is the product of the
    // singular values. For a square J that is just |det J|.
    m.yoshikawa = m.singular_values.prod();
    m.sigma_max = m.singular_values.size() ? m.singular_values(0) : 0.0;
    m.sigma_min = m.singular_values.size() ? m.singular_values(m.singular_values.size() - 1) : 0.0;
    m.condition_number = m.sigma_min > 0.0 ? m.sigma_max / m.sigma_min
                                           : std::numeric_limits<double>::infinity();
    return m;
}

double wrapAngle(double a) {
    double x = std::fmod(a + M_PI, 2.0 * M_PI);
    if (x <= 0.0) x += 2.0 * M_PI;
    return x - M_PI;
}

Eigen::VectorXd wrapAngles(const Eigen::VectorXd& q) {
    Eigen::VectorXd out(q.size());
    for (Eigen::Index i = 0; i < q.size(); ++i) out[i] = wrapAngle(q[i]);
    return out;
}

double angleDiff(double a, double b) { return wrapAngle(a - b); }

Eigen::VectorXd angleDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    Eigen::VectorXd out(a.size());
    for (Eigen::Index i = 0; i < a.size(); ++i) out[i] = wrapAngle(a[i] - b[i]);
    return out;
}

}  // namespace arm
