#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "arm/collision.hpp"

using Catch::Approx;
using namespace arm;

namespace {

Eigen::Vector2d p(double x, double y) { return Eigen::Vector2d(x, y); }

Eigen::VectorXd vec(std::initializer_list<double> v) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(v.size()));
    Eigen::Index i = 0;
    for (double x : v) q[i++] = x;
    return q;
}

ArmModel make3R(double radius) {
    Eigen::VectorXd lengths(3);
    lengths << 1.0, 1.0, 1.0;
    ArmModel model(lengths);
    model.link_radius = radius;
    return model;
}

}  // namespace

TEST_CASE("point to segment distance clamps to the endpoints", "[collision][primitives]") {
    CHECK(pointSegmentDistance(p(0.5, 1.0), p(0, 0), p(1, 0)) == Approx(1.0));
    CHECK(pointSegmentDistance(p(-1.0, 0.0), p(0, 0), p(1, 0)) == Approx(1.0));
    CHECK(pointSegmentDistance(p(2.0, 0.0), p(0, 0), p(1, 0)) == Approx(1.0));
    CHECK(pointSegmentDistance(p(0.5, 0.0), p(0, 0), p(1, 0)) == Approx(0.0).margin(1e-12));
    // Degenerate segment.
    CHECK(pointSegmentDistance(p(3.0, 4.0), p(0, 0), p(0, 0)) == Approx(5.0));
}

TEST_CASE("segment intersection", "[collision][primitives]") {
    CHECK(segmentsIntersect(p(-1, 0), p(1, 0), p(0, -1), p(0, 1)));
    CHECK_FALSE(segmentsIntersect(p(-1, 0), p(-0.1, 0), p(0, -1), p(0, 1)));
    CHECK_FALSE(segmentsIntersect(p(0, 0), p(1, 0), p(0, 1), p(1, 1)));  // parallel
    CHECK(segmentsIntersect(p(0, 0), p(2, 0), p(1, 0), p(3, 0)));        // collinear overlap
    CHECK_FALSE(segmentsIntersect(p(0, 0), p(1, 0), p(2, 0), p(3, 0)));  // collinear, disjoint
    CHECK(segmentsIntersect(p(0, 0), p(1, 0), p(1, 0), p(1, 1)));        // touching endpoints
}

TEST_CASE("segment to segment distance", "[collision][primitives]") {
    CHECK(segmentSegmentDistance(p(0, 0), p(1, 0), p(0, 1), p(1, 1)) == Approx(1.0));
    CHECK(segmentSegmentDistance(p(-1, 0), p(1, 0), p(0, -1), p(0, 1)) == Approx(0.0));
    CHECK(segmentSegmentDistance(p(0, 0), p(1, 0), p(2, 1), p(3, 1)) == Approx(std::sqrt(2.0)));
}

TEST_CASE("segment against an axis aligned box", "[collision][primitives]") {
    const Aabb box{p(1, 1), p(2, 2)};

    CHECK(box.contains(p(1.5, 1.5)));
    CHECK_FALSE(box.contains(p(0.5, 1.5)));
    CHECK(segmentIntersectsAabb(p(0, 1.5), p(3, 1.5), box));
    CHECK(segmentIntersectsAabb(p(1.5, 1.5), p(1.6, 1.6), box));  // fully inside
    CHECK_FALSE(segmentIntersectsAabb(p(0, 0), p(0, 3), box));
    CHECK(segmentAabbDistance(p(0, 1.5), p(3, 1.5), box) == Approx(0.0));
    CHECK(segmentAabbDistance(p(0, 1.5), p(0.5, 1.5), box) == Approx(0.5));
    CHECK(segmentAabbDistance(p(0, 0), p(1, 0), box) == Approx(1.0));
}

TEST_CASE("capsules against obstacles", "[collision][primitives]") {
    const Circle c{p(0.5, 0.4), 0.2};
    CHECK_FALSE(capsuleHitsCircle(p(0, 0), p(1, 0), 0.1, c));
    CHECK(capsuleHitsCircle(p(0, 0), p(1, 0), 0.25, c));

    const Aabb box{p(0.4, 0.3), p(0.6, 0.5)};
    CHECK_FALSE(capsuleHitsAabb(p(0, 0), p(1, 0), 0.2, box));
    CHECK(capsuleHitsAabb(p(0, 0), p(1, 0), 0.35, box));
}

TEST_CASE("the arm reports which link hit which obstacle", "[collision][arm]") {
    const ArmModel model = make3R(0.05);
    World world;
    world.circles.push_back(Circle{p(2.5, 0.0), 0.2});  // sits on link 3

    const Eigen::VectorXd straight = vec({0.0, 0.0, 0.0});
    const CollisionReport rep = checkObstacles(model, straight, world);
    REQUIRE(rep.hit());
    CHECK(rep.kind == CollisionReport::Kind::Obstacle);
    CHECK(rep.link == 2);
    CHECK(rep.obstacle == 0);
    CHECK(rep.obstacle_is_circle);

    SECTION("folding the arm away clears it") {
        CHECK_FALSE(checkObstacles(model, vec({M_PI / 2.0, 0.0, 0.0}), world).hit());
    }
    SECTION("boxes are reported as boxes") {
        World boxes;
        boxes.boxes.push_back(Aabb{p(0.4, -0.1), p(0.6, 0.1)});
        const CollisionReport b = checkObstacles(model, straight, boxes);
        REQUIRE(b.hit());
        CHECK(b.link == 0);
        CHECK_FALSE(b.obstacle_is_circle);
    }
    SECTION("an empty world never collides") {
        CHECK_FALSE(checkObstacles(model, straight, World{}).hit());
    }
}

TEST_CASE("self collision only looks at non-adjacent links", "[collision][arm]") {
    const ArmModel model = make3R(0.02);

    // Adjacent links always touch at the shared joint, so a sharply bent
    // elbow is not a self collision.
    CHECK_FALSE(checkSelfCollision(model, vec({0.0, 2.9, 0.0})).hit());
    CHECK_FALSE(checkSelfCollision(model, vec({0.0, 0.0, 0.0})).hit());

    // Curled far enough that link 3 crosses back over link 1.
    const CollisionReport rep = checkSelfCollision(model, vec({0.0, 3.0, 3.0}));
    REQUIRE(rep.hit());
    CHECK(rep.kind == CollisionReport::Kind::Self);
    CHECK(rep.link == 0);
    CHECK(rep.other_link == 2);
}

TEST_CASE("path checking catches collisions the endpoints miss", "[collision][path]") {
    const ArmModel model = make3R(0.05);
    World world;
    // A post the arm has to sweep past on its way from one side to the other.
    world.circles.push_back(Circle{p(0.0, 2.0), 0.3});

    const Eigen::VectorXd start = vec({0.0, 0.0, 0.0});          // pointing +x
    const Eigen::VectorXd goal = vec({M_PI, 0.0, 0.0});          // pointing -x
    CHECK_FALSE(checkCollision(model, start, world).hit());
    CHECK_FALSE(checkCollision(model, goal, world).hit());

    const CollisionReport rep = checkPath(model, start, goal, world, 64);
    REQUIRE(rep.hit());
    CHECK(rep.kind == CollisionReport::Kind::Obstacle);

    SECTION("a clear path stays clear") {
        CHECK_FALSE(checkPath(model, start, vec({-0.5, 0.2, 0.1}), world, 64).hit());
    }
}
