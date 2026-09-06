#include "arm/ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm {
namespace {

double clampUnit(double x) { return std::min(1.0, std::max(-1.0, x)); }

/// Two joint angles as a vector, wrapped into (-pi, pi].
Eigen::VectorXd pack2(double t1, double t2) {
    Eigen::VectorXd q(2);
    q << wrapAngle(t1), wrapAngle(t2);
    return q;
}

}  // namespace

AnalyticIkResult analyticIk2R(const ArmModel& model, const Eigen::Vector2d& target, double tol) {
    if (model.dof() != 2) throw std::invalid_argument("analyticIk2R: model is not 2R");

    const double l1 = model.link_lengths[0];
    const double l2 = model.link_lengths[1];
    const Eigen::Vector2d p = model.base.inverse() * target;
    const double r = p.norm();
    const double outer = l1 + l2;
    const double inner = std::abs(l1 - l2);

    AnalyticIkResult out;
    if (r > outer + tol || r < inner - tol) return out;  // outside the annulus
    out.reachable = true;

    // Equal links and a target at the base: the shoulder is free. Report one
    // representative folded solution rather than pretending it is unique.
    if (r <= tol && inner <= tol) {
        out.at_boundary = true;
        out.solutions.push_back(pack2(0.0, M_PI));
        return out;
    }

    const double c2 = clampUnit((r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2));
    const double t2 = std::acos(c2);
    const double base_angle = std::atan2(p.y(), p.x());

    // At full extension or the inner boundary the two branches coincide.
    out.at_boundary = std::abs(std::abs(c2) - 1.0) < 1e-12;

    auto branch = [&](double elbow) {
        const double t1 = base_angle - std::atan2(l2 * std::sin(elbow), l1 + l2 * std::cos(elbow));
        out.solutions.push_back(pack2(t1, elbow));
    };
    branch(t2);
    if (!out.at_boundary) branch(-t2);
    return out;
}

AnalyticIkResult analyticIk3R(const ArmModel& model, const Eigen::Vector2d& target,
                              double ee_orientation, double tol) {
    if (model.dof() != 3) throw std::invalid_argument("analyticIk3R: model is not 3R");

    // The heading pins the wrist centre, which leaves a 2R position problem.
    const double phi = ee_orientation;
    const Eigen::Vector2d wrist =
        target - model.link_lengths[2] * Eigen::Vector2d(std::cos(phi), std::sin(phi));

    ArmModel sub;
    sub.link_lengths = model.link_lengths.head(2);
    sub.joint_min = model.joint_min.head(2);
    sub.joint_max = model.joint_max.head(2);
    sub.base = model.base;

    const AnalyticIkResult sub_result = analyticIk2R(sub, wrist, tol);
    const double base_angle = Eigen::Rotation2Dd(model.base.linear()).angle();

    AnalyticIkResult out;
    out.reachable = sub_result.reachable;
    out.at_boundary = sub_result.at_boundary;
    for (const Eigen::VectorXd& s : sub_result.solutions) {
        Eigen::VectorXd q(3);
        q << s[0], s[1], wrapAngle(phi - base_angle - s[0] - s[1]);
        out.solutions.push_back(q);
    }
    return out;
}

AnalyticIkResult analyticIk(const ArmModel& model, const Eigen::Vector2d& target,
                            std::optional<double> ee_orientation, double tol) {
    if (model.dof() == 2) return analyticIk2R(model, target, tol);
    if (model.dof() == 3) {
        if (!ee_orientation) {
            throw std::invalid_argument("analyticIk: 3R needs an end effector orientation");
        }
        return analyticIk3R(model, target, *ee_orientation, tol);
    }
    throw std::invalid_argument("analyticIk: no closed form for this chain");
}

AnalyticIkResult filterToLimits(const ArmModel& model, const AnalyticIkResult& in, double tol) {
    AnalyticIkResult out;
    out.reachable = in.reachable;
    out.at_boundary = in.at_boundary;
    for (const Eigen::VectorXd& q : in.solutions) {
        if (model.withinLimits(q, tol)) out.solutions.push_back(q);
    }
    return out;
}

std::optional<Eigen::VectorXd> nearestSolution(const AnalyticIkResult& in,
                                               const Eigen::VectorXd& reference) {
    std::optional<Eigen::VectorXd> best;
    double best_cost = 0.0;
    for (const Eigen::VectorXd& q : in.solutions) {
        const double cost = angleDiff(q, reference).norm();
        if (!best || cost < best_cost) {
            best = q;
            best_cost = cost;
        }
    }
    return best;
}

DlsResult dampedLeastSquaresIk(const ArmModel& model, const Eigen::Vector2d& target,
                               const Eigen::VectorXd& q_seed, const DlsOptions& opts) {
    const Eigen::Index rows = opts.solve_orientation ? 3 : 2;
    Eigen::VectorXd q = q_seed;
    DlsResult result;

    auto residual = [&](const FkResult& fk) {
        Eigen::VectorXd e(rows);
        e.head(2) = target - fk.eePosition();
        if (opts.solve_orientation) e[2] = wrapAngle(opts.target_orientation - fk.eeOrientation());
        return e;
    };

    FkResult fk = forwardKinematics(model, q);
    Eigen::VectorXd e = residual(fk);

    for (int it = 0; it < opts.max_iterations; ++it) {
        if (e.norm() < opts.tolerance) break;
        result.iterations = it + 1;

        const Eigen::MatrixXd J = jacobian(fk).topRows(rows);
        const Manipulability m = manipulability(J);

        // Adaptive damping: none worth speaking of in the well conditioned
        // interior, ramped up as sigma_min collapses so the step near a
        // singularity stays bounded instead of exploding.
        double lambda_sq = opts.lambda * opts.lambda;
        if (opts.adaptive && m.sigma_min < opts.sigma_threshold) {
            const double ratio = m.sigma_min / opts.sigma_threshold;
            lambda_sq += opts.lambda_max * opts.lambda_max * (1.0 - ratio * ratio);
        }

        const Eigen::MatrixXd A =
            J * J.transpose() + lambda_sq * Eigen::MatrixXd::Identity(rows, rows);
        Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(e);

        const double step = dq.norm();
        if (opts.max_step > 0.0 && step > opts.max_step) dq *= opts.max_step / step;

        q = wrapAngles(q + dq);
        if (opts.respect_limits) q = model.clampToLimits(q);

        fk = forwardKinematics(model, q);
        e = residual(fk);
    }

    result.q = q;
    result.error = e.norm();
    result.converged = result.error < opts.tolerance;
    result.sigma_min = manipulability(jacobian(fk).topRows(rows)).sigma_min;
    return result;
}

}  // namespace arm
