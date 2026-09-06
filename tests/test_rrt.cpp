#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "arm/rrt.hpp"

using Catch::Approx;
using namespace arm;

namespace {

ArmModel make3R() {
    Eigen::VectorXd lengths(3);
    lengths << 1.0, 0.8, 0.6;
    ArmModel model(lengths, 2.4);
    model.link_radius = 0.04;
    return model;
}

Eigen::VectorXd vec(std::initializer_list<double> v) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(v.size()));
    Eigen::Index i = 0;
    for (double x : v) q[i++] = x;
    return q;
}

}  // namespace

TEST_CASE("a clear straight line needs no tree", "[rrt]") {
    const ArmModel model = make3R();
    const RrtResult r = planRrt(model, vec({0.0, 0.3, 0.0}), vec({0.4, 0.5, 0.2}), World{});
    REQUIRE(r.success);
    REQUIRE(r.path.size() == 2);
    CHECK(r.iterations == 0);
}

TEST_CASE("the planner routes around an obstacle", "[rrt]") {
    const ArmModel model = make3R();
    World world;
    // A post right where the straight line in joint space would sweep.
    world.circles.push_back(Circle{Eigen::Vector2d(1.6, 0.0), 0.35});

    const Eigen::VectorXd start = vec({1.0, 0.2, 0.1});
    const Eigen::VectorXd goal = vec({-1.0, -0.2, -0.1});
    REQUIRE_FALSE(checkCollision(model, start, world).hit());
    REQUIRE_FALSE(checkCollision(model, goal, world).hit());
    REQUIRE(checkPath(model, start, goal, world, 32).hit());

    RrtConfig cfg;
    cfg.seed = 7;
    cfg.max_iterations = 8000;
    const RrtResult r = planRrt(model, start, goal, world, cfg);

    REQUIRE(r.success);
    REQUIRE(r.path.size() >= 2);
    CHECK(angleDiff(r.path.front(), start).norm() == Approx(0.0).margin(1e-12));
    CHECK(angleDiff(r.path.back(), goal).norm() == Approx(0.0).margin(1e-12));

    // Every edge of the returned path has to be clear, not just the waypoints.
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
        CHECK(model.withinLimits(r.path[i]));
        CHECK_FALSE(checkPath(model, r.path[i], r.path[i + 1], world, 32).hit());
    }
}

TEST_CASE("planning from or to a colliding configuration fails cleanly", "[rrt]") {
    const ArmModel model = make3R();
    World world;
    world.circles.push_back(Circle{Eigen::Vector2d(1.2, 0.0), 0.5});

    const Eigen::VectorXd blocked = vec({0.0, 0.0, 0.0});
    REQUIRE(checkCollision(model, blocked, world).hit());

    const RrtResult from_blocked = planRrt(model, blocked, vec({1.5, 0.5, 0.0}), world);
    CHECK_FALSE(from_blocked.success);
    CHECK(from_blocked.path.empty());

    const RrtResult to_blocked = planRrt(model, vec({1.5, 0.5, 0.0}), blocked, world);
    CHECK_FALSE(to_blocked.success);
}

TEST_CASE("the same seed plans the same path", "[rrt]") {
    const ArmModel model = make3R();
    World world;
    world.circles.push_back(Circle{Eigen::Vector2d(1.6, 0.0), 0.35});

    RrtConfig cfg;
    cfg.seed = 99;
    cfg.max_iterations = 8000;
    const Eigen::VectorXd start = vec({1.0, 0.2, 0.1});
    const Eigen::VectorXd goal = vec({-1.0, -0.2, -0.1});

    const RrtResult a = planRrt(model, start, goal, world, cfg);
    const RrtResult b = planRrt(model, start, goal, world, cfg);
    REQUIRE(a.success);
    REQUIRE(a.path.size() == b.path.size());
    for (std::size_t i = 0; i < a.path.size(); ++i) {
        CHECK((a.path[i] - b.path[i]).norm() == Approx(0.0).margin(1e-15));
    }

    cfg.seed = 100;
    const RrtResult c = planRrt(model, start, goal, world, cfg);
    CHECK(c.success);
    CHECK(c.length() > 0.0);
}
