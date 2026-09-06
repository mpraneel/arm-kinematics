#include "arm/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm {
namespace {

double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

}  // namespace

bool Aabb::contains(const Eigen::Vector2d& p) const {
    return p.x() >= min.x() && p.x() <= max.x() && p.y() >= min.y() && p.y() <= max.y();
}

bool World::occupied(const Eigen::Vector2d& p, double margin) const {
    for (const Circle& c : circles) {
        if ((p - c.center).norm() <= c.radius + margin) return true;
    }
    for (const Aabb& b : boxes) {
        Aabb grown{b.min - Eigen::Vector2d::Constant(margin),
                   b.max + Eigen::Vector2d::Constant(margin)};
        if (grown.contains(p)) return true;
    }
    return false;
}

double pointSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b) {
    const Eigen::Vector2d ab = b - a;
    const double len_sq = ab.squaredNorm();
    if (len_sq < 1e-18) return (p - a).norm();
    const double t = std::min(1.0, std::max(0.0, (p - a).dot(ab) / len_sq));
    return (p - (a + t * ab)).norm();
}

bool segmentsIntersect(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1,
                       const Eigen::Vector2d& b0, const Eigen::Vector2d& b1) {
    const Eigen::Vector2d r = a1 - a0;
    const Eigen::Vector2d s = b1 - b0;
    const double denom = cross2(r, s);
    const double num = cross2(b0 - a0, r);

    if (std::abs(denom) < 1e-18) {
        if (std::abs(num) > 1e-18) return false;  // parallel, not collinear
        // Collinear: overlap along the shared direction.
        const double rr = r.dot(r);
        if (rr < 1e-18) return pointSegmentDistance(a0, b0, b1) < 1e-12;
        const double t0 = (b0 - a0).dot(r) / rr;
        const double t1 = (b1 - a0).dot(r) / rr;
        return std::max(0.0, std::min(t0, t1)) <= std::min(1.0, std::max(t0, t1));
    }
    const double t = cross2(b0 - a0, s) / denom;
    const double u = num / denom;
    return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
}

double segmentSegmentDistance(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1,
                              const Eigen::Vector2d& b0, const Eigen::Vector2d& b1) {
    if (segmentsIntersect(a0, a1, b0, b1)) return 0.0;
    return std::min({pointSegmentDistance(a0, b0, b1), pointSegmentDistance(a1, b0, b1),
                     pointSegmentDistance(b0, a0, a1), pointSegmentDistance(b1, a0, a1)});
}

bool segmentIntersectsAabb(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Aabb& box) {
    // Liang-Barsky slab clipping.
    const Eigen::Vector2d d = b - a;
    double t0 = 0.0, t1 = 1.0;
    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(d[axis]) < 1e-18) {
            if (a[axis] < box.min[axis] || a[axis] > box.max[axis]) return false;
            continue;
        }
        double tn = (box.min[axis] - a[axis]) / d[axis];
        double tf = (box.max[axis] - a[axis]) / d[axis];
        if (tn > tf) std::swap(tn, tf);
        t0 = std::max(t0, tn);
        t1 = std::min(t1, tf);
        if (t0 > t1) return false;
    }
    return true;
}

double segmentAabbDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Aabb& box) {
    if (segmentIntersectsAabb(a, b, box)) return 0.0;
    const Eigen::Vector2d c0(box.min.x(), box.min.y());
    const Eigen::Vector2d c1(box.max.x(), box.min.y());
    const Eigen::Vector2d c2(box.max.x(), box.max.y());
    const Eigen::Vector2d c3(box.min.x(), box.max.y());
    return std::min({segmentSegmentDistance(a, b, c0, c1), segmentSegmentDistance(a, b, c1, c2),
                     segmentSegmentDistance(a, b, c2, c3), segmentSegmentDistance(a, b, c3, c0)});
}

bool capsuleHitsCircle(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double radius,
                       const Circle& c) {
    return pointSegmentDistance(c.center, a, b) <= c.radius + radius;
}

bool capsuleHitsAabb(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double radius,
                     const Aabb& box) {
    return segmentAabbDistance(a, b, box) <= radius;
}

CollisionReport checkObstacles(const ArmModel& model, const Eigen::VectorXd& q, const World& world,
                               double margin) {
    const std::vector<Eigen::Vector2d> pts = forwardKinematics(model, q).points();
    const double r = model.link_radius + margin;

    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        for (std::size_t j = 0; j < world.circles.size(); ++j) {
            if (capsuleHitsCircle(pts[i], pts[i + 1], r, world.circles[j])) {
                CollisionReport rep;
                rep.kind = CollisionReport::Kind::Obstacle;
                rep.link = static_cast<int>(i);
                rep.obstacle = static_cast<int>(j);
                rep.obstacle_is_circle = true;
                return rep;
            }
        }
        for (std::size_t j = 0; j < world.boxes.size(); ++j) {
            if (capsuleHitsAabb(pts[i], pts[i + 1], r, world.boxes[j])) {
                CollisionReport rep;
                rep.kind = CollisionReport::Kind::Obstacle;
                rep.link = static_cast<int>(i);
                rep.obstacle = static_cast<int>(j);
                rep.obstacle_is_circle = false;
                return rep;
            }
        }
    }
    return {};
}

CollisionReport checkSelfCollision(const ArmModel& model, const Eigen::VectorXd& q, double margin) {
    const std::vector<Eigen::Vector2d> pts = forwardKinematics(model, q).points();
    const double clearance = 2.0 * model.link_radius + margin;
    const std::size_t links = pts.size() - 1;

    for (std::size_t i = 0; i + 2 < links; ++i) {
        for (std::size_t j = i + 2; j < links; ++j) {
            if (segmentSegmentDistance(pts[i], pts[i + 1], pts[j], pts[j + 1]) <= clearance) {
                CollisionReport rep;
                rep.kind = CollisionReport::Kind::Self;
                rep.link = static_cast<int>(i);
                rep.other_link = static_cast<int>(j);
                return rep;
            }
        }
    }
    return {};
}

CollisionReport checkCollision(const ArmModel& model, const Eigen::VectorXd& q, const World& world,
                               double margin) {
    CollisionReport rep = checkObstacles(model, q, world, margin);
    if (rep.hit()) return rep;
    return checkSelfCollision(model, q, margin);
}

CollisionReport checkPath(const ArmModel& model, const Eigen::VectorXd& q0,
                          const Eigen::VectorXd& q1, const World& world, int steps, double margin) {
    const int n = std::max(1, steps);
    // Interpolate along the shortest wrapped path, which is what the arm
    // actually takes.
    const Eigen::VectorXd delta = angleDiff(q1, q0);
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        CollisionReport rep = checkCollision(model, wrapAngles(q0 + t * delta), world, margin);
        if (rep.hit()) return rep;
    }
    return {};
}

}  // namespace arm
