#include "arm/rrt.hpp"

#include <algorithm>
#include <limits>
#include <random>

namespace arm {
namespace {

/// Straight line in joint space, checked at every waypoint.
bool edgeIsClear(const ArmModel& model, const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                 const World& world, const RrtConfig& cfg) {
    return !checkPath(model, a, b, world, cfg.edge_steps, cfg.collision_margin).hit();
}

}  // namespace

double RrtResult::length() const {
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) total += (path[i + 1] - path[i]).norm();
    return total;
}

RrtResult planRrt(const ArmModel& model, const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                  const World& world, const RrtConfig& cfg) {
    RrtResult result;
    const Eigen::Index n = start.size();

    if (checkCollision(model, start, world, cfg.collision_margin).hit()) return result;
    if (checkCollision(model, goal, world, cfg.collision_margin).hit()) return result;

    // Trivial case: the straight line already works.
    if (edgeIsClear(model, start, goal, world, cfg)) {
        result.path = {start, goal};
        result.success = true;
        result.nodes = 2;
        return result;
    }

    std::mt19937_64 rng(cfg.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    std::vector<Eigen::VectorXd> nodes{start};
    std::vector<int> parent{-1};

    auto sample = [&]() {
        if (unit(rng) < cfg.goal_bias) return goal;
        Eigen::VectorXd q(n);
        for (Eigen::Index i = 0; i < n; ++i) {
            q[i] = model.joint_min[i] + unit(rng) * (model.joint_max[i] - model.joint_min[i]);
        }
        return q;
    };

    int goal_index = -1;
    for (int it = 0; it < cfg.max_iterations && goal_index < 0; ++it) {
        result.iterations = it + 1;
        const Eigen::VectorXd target = sample();

        int nearest = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const double d = (nodes[i] - target).squaredNorm();
            if (d < best) {
                best = d;
                nearest = static_cast<int>(i);
            }
        }

        const Eigen::VectorXd& from = nodes[static_cast<std::size_t>(nearest)];
        Eigen::VectorXd delta = target - from;
        const double dist = delta.norm();
        if (dist < 1e-9) continue;
        const Eigen::VectorXd to = from + delta * (std::min(cfg.step, dist) / dist);

        if (!model.withinLimits(to)) continue;
        if (!edgeIsClear(model, from, to, world, cfg)) continue;

        nodes.push_back(to);
        parent.push_back(nearest);

        if ((to - goal).norm() <= cfg.goal_tolerance &&
            edgeIsClear(model, to, goal, world, cfg)) {
            nodes.push_back(goal);
            parent.push_back(static_cast<int>(nodes.size()) - 2);
            goal_index = static_cast<int>(nodes.size()) - 1;
        }
    }

    result.nodes = static_cast<int>(nodes.size());
    if (goal_index < 0) return result;

    for (int i = goal_index; i >= 0; i = parent[static_cast<std::size_t>(i)]) {
        result.path.push_back(nodes[static_cast<std::size_t>(i)]);
    }
    std::reverse(result.path.begin(), result.path.end());
    result.success = true;

    if (cfg.shortcut && result.path.size() > 2) {
        for (int attempt = 0; attempt < cfg.shortcut_attempts && result.path.size() > 2; ++attempt) {
            std::uniform_int_distribution<std::size_t> idx(0, result.path.size() - 1);
            std::size_t i = idx(rng), j = idx(rng);
            if (i > j) std::swap(i, j);
            if (j <= i + 1) continue;
            if (!edgeIsClear(model, result.path[i], result.path[j], world, cfg)) continue;
            result.path.erase(result.path.begin() + static_cast<long>(i) + 1,
                              result.path.begin() + static_cast<long>(j));
        }
    }
    return result;
}

}  // namespace arm
