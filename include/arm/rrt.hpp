#pragma once

#include <cstdint>
#include <vector>

#include "arm/collision.hpp"
#include "arm/kinematics.hpp"

namespace arm {

struct RrtConfig {
    int max_iterations = 3000;
    /// Extension step in joint space, radians.
    double step = 0.25;
    /// How often the sampler picks the goal instead of a random configuration.
    double goal_bias = 0.1;
    /// A node this close to the goal ends the search.
    double goal_tolerance = 0.15;
    /// Waypoints used when collision checking one extension.
    int edge_steps = 8;
    double collision_margin = 0.0;
    /// Drop waypoints that a straight shortcut can replace.
    bool shortcut = true;
    int shortcut_attempts = 60;
    std::uint64_t seed = 1;
};

struct RrtResult {
    /// Start to goal inclusive, empty when the search failed.
    std::vector<Eigen::VectorXd> path;
    bool success = false;
    int iterations = 0;
    int nodes = 0;

    /// Total joint space length of the path.
    double length() const;
};

/// Sampling based planner on the joint space of the arm, which for a 3R chain
/// is a real C-space rather than a toy. Samples respect the joint limits and
/// every edge is collision checked, not just the endpoints.
RrtResult planRrt(const ArmModel& model, const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                  const World& world, const RrtConfig& config = {});

}  // namespace arm
