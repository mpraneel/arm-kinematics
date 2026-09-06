#pragma once

#include <vector>

#include "arm/kinematics.hpp"

namespace arm {

struct Circle {
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double radius = 0.0;
};

/// Axis aligned box, min corner and max corner.
struct Aabb {
    Eigen::Vector2d min = Eigen::Vector2d::Zero();
    Eigen::Vector2d max = Eigen::Vector2d::Zero();

    bool contains(const Eigen::Vector2d& p) const;
};

/// The static scene the arm has to avoid.
struct World {
    std::vector<Circle> circles;
    std::vector<Aabb> boxes;

    bool empty() const { return circles.empty() && boxes.empty(); }
    /// True if the point is inside any obstacle, optionally inflated by margin.
    bool occupied(const Eigen::Vector2d& p, double margin = 0.0) const;
};

double pointSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b);
double segmentSegmentDistance(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1,
                              const Eigen::Vector2d& b0, const Eigen::Vector2d& b1);
bool segmentsIntersect(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1,
                       const Eigen::Vector2d& b0, const Eigen::Vector2d& b1);
bool segmentIntersectsAabb(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Aabb& box);
double segmentAabbDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Aabb& box);

/// A link is a capsule: the segment a-b swept by `radius`.
bool capsuleHitsCircle(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double radius,
                       const Circle& c);
bool capsuleHitsAabb(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double radius,
                     const Aabb& box);

struct CollisionReport {
    enum class Kind { None, Obstacle, Self };

    Kind kind = Kind::None;
    /// Colliding link, and for a self collision the second link too.
    int link = -1;
    int other_link = -1;
    /// Index into World::circles or World::boxes, whichever `circle` says.
    int obstacle = -1;
    bool obstacle_is_circle = false;

    bool hit() const { return kind != Kind::None; }
    explicit operator bool() const { return hit(); }
};

/// First obstacle collision found, links treated as capsules of
/// `model.link_radius` inflated by `margin`.
CollisionReport checkObstacles(const ArmModel& model, const Eigen::VectorXd& q, const World& world,
                               double margin = 0.0);

/// First collision between two non-adjacent links. Adjacent links always touch
/// at their shared joint, so they are skipped by construction.
CollisionReport checkSelfCollision(const ArmModel& model, const Eigen::VectorXd& q,
                                   double margin = 0.0);

/// Obstacles first, then self collision.
CollisionReport checkCollision(const ArmModel& model, const Eigen::VectorXd& q, const World& world,
                               double margin = 0.0);

/// Endpoint checks miss the middle of a motion, so the straight joint space
/// path from `q0` to `q1` is discretized into `steps` intervals and every
/// waypoint is checked. Returns the first colliding waypoint's report.
CollisionReport checkPath(const ArmModel& model, const Eigen::VectorXd& q0,
                          const Eigen::VectorXd& q1, const World& world, int steps = 16,
                          double margin = 0.0);

}  // namespace arm
