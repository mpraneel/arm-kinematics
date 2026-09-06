#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <random>

#include "arm/ik.hpp"

using Catch::Approx;
using namespace arm;

namespace {

ArmModel make2R(double l1 = 1.0, double l2 = 0.7) {
    Eigen::VectorXd lengths(2);
    lengths << l1, l2;
    return ArmModel(lengths);
}

ArmModel make3R() {
    Eigen::VectorXd lengths(3);
    lengths << 1.0, 0.8, 0.5;
    return ArmModel(lengths);
}

Eigen::VectorXd vec(std::initializer_list<double> v) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(v.size()));
    Eigen::Index i = 0;
    for (double x : v) q[i++] = x;
    return q;
}

bool containsConfig(const AnalyticIkResult& r, const Eigen::VectorXd& q, double tol = 1e-9) {
    for (const Eigen::VectorXd& s : r.solutions) {
        if (angleDiff(s, q).norm() < tol) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("2R analytic IK returns both elbow branches", "[ik][analytic]") {
    const ArmModel model = make2R();
    const AnalyticIkResult r = analyticIk2R(model, Eigen::Vector2d(0.9, 0.6));

    REQUIRE(r.reachable);
    REQUIRE(r.size() == 2);
    CHECK_FALSE(r.at_boundary);
    // Elbow up and elbow down: the elbow angles are opposite in sign.
    CHECK(r.solutions[0][1] == Approx(-r.solutions[1][1]));
    for (const Eigen::VectorXd& q : r.solutions) {
        const Eigen::Vector2d p = eePosition(model, q);
        CHECK(p.x() == Approx(0.9).margin(1e-12));
        CHECK(p.y() == Approx(0.6).margin(1e-12));
    }
}

TEST_CASE("2R analytic IK is exact on the annulus boundaries", "[ik][analytic]") {
    const ArmModel model = make2R(1.0, 0.7);

    SECTION("full extension gives one solution") {
        const AnalyticIkResult r = analyticIk2R(model, Eigen::Vector2d(1.7, 0.0));
        REQUIRE(r.reachable);
        CHECK(r.at_boundary);
        REQUIRE(r.size() == 1);
        CHECK(r.solutions[0][1] == Approx(0.0).margin(1e-6));
    }
    SECTION("the inner hole boundary gives one folded solution") {
        const AnalyticIkResult r = analyticIk2R(model, Eigen::Vector2d(0.0, 0.3));
        REQUIRE(r.reachable);
        CHECK(r.at_boundary);
        REQUIRE(r.size() == 1);
        CHECK(std::abs(r.solutions[0][1]) == Approx(M_PI).margin(1e-6));
    }
    SECTION("outside the annulus there are no solutions") {
        CHECK(analyticIk2R(model, Eigen::Vector2d(2.0, 0.0)).empty());
        CHECK_FALSE(analyticIk2R(model, Eigen::Vector2d(2.0, 0.0)).reachable);
        CHECK(analyticIk2R(model, Eigen::Vector2d(0.1, 0.0)).empty());
    }
    SECTION("equal links can reach the base, where the shoulder is free") {
        const ArmModel equal = make2R(1.0, 1.0);
        const AnalyticIkResult r = analyticIk2R(equal, Eigen::Vector2d(0.0, 0.0));
        REQUIRE(r.reachable);
        REQUIRE(r.size() == 1);
        CHECK(eePosition(equal, r.solutions[0]).norm() == Approx(0.0).margin(1e-12));
    }
}

TEST_CASE("FK then analytic IK round trips for random reachable targets", "[ik][analytic]") {
    const ArmModel model = make2R();
    std::mt19937 rng(20240607);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    for (int trial = 0; trial < 500; ++trial) {
        Eigen::VectorXd q(2);
        q << angle(rng), angle(rng);
        const Eigen::Vector2d target = eePosition(model, q);

        const AnalyticIkResult r = analyticIk2R(model, target);
        REQUIRE(r.reachable);
        // The original configuration has to be one of the branches.
        CHECK(containsConfig(r, q, 1e-7));
        for (const Eigen::VectorXd& s : r.solutions) {
            CHECK((eePosition(model, s) - target).norm() == Approx(0.0).margin(1e-9));
        }
    }
}

TEST_CASE("3R analytic IK hits the position and the heading", "[ik][analytic]") {
    const ArmModel model = make3R();
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    for (int trial = 0; trial < 300; ++trial) {
        Eigen::VectorXd q(3);
        for (int i = 0; i < 3; ++i) q[i] = angle(rng);
        const FkResult fk = forwardKinematics(model, q);

        const AnalyticIkResult r = analyticIk3R(model, fk.eePosition(), fk.eeOrientation());
        REQUIRE(r.reachable);
        CHECK(containsConfig(r, q, 1e-7));
        for (const Eigen::VectorXd& s : r.solutions) {
            const FkResult check = forwardKinematics(model, s);
            CHECK((check.eePosition() - fk.eePosition()).norm() == Approx(0.0).margin(1e-9));
            CHECK(angleDiff(check.eeOrientation(), fk.eeOrientation()) == Approx(0.0).margin(1e-9));
        }
    }
}

TEST_CASE("a base pose does not confuse the analytic solvers", "[ik][analytic]") {
    ArmModel model = make3R();
    model.base.translation() = Eigen::Vector2d(-0.5, 0.75);
    model.base.linear() = Eigen::Rotation2Dd(0.6).toRotationMatrix();

    const Eigen::VectorXd q = vec({0.4, -0.9, 1.1});
    const FkResult fk = forwardKinematics(model, q);
    const AnalyticIkResult r = analyticIk(model, fk.eePosition(), fk.eeOrientation());

    REQUIRE(r.reachable);
    CHECK(containsConfig(r, q, 1e-7));
}

TEST_CASE("solutions can be filtered by joint limits and ranked", "[ik][analytic]") {
    ArmModel model = make2R();
    // Elbow-down only.
    model.joint_min << -M_PI, -M_PI;
    model.joint_max << M_PI, 0.0;

    const AnalyticIkResult all = analyticIk2R(model, Eigen::Vector2d(0.9, 0.6));
    REQUIRE(all.size() == 2);
    const AnalyticIkResult legal = filterToLimits(model, all);
    REQUIRE(legal.size() == 1);
    CHECK(legal.solutions[0][1] <= 0.0);

    const auto nearest = nearestSolution(all, vec({0.0, 1.0}));
    REQUIRE(nearest.has_value());
    CHECK((*nearest)[1] > 0.0);
    CHECK_FALSE(nearestSolution(AnalyticIkResult{}, vec({0.0, 0.0})).has_value());
}

TEST_CASE("damped least squares converges on reachable targets", "[ik][dls]") {
    const ArmModel model = make3R();
    std::mt19937 rng(99);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    DlsOptions opts;
    opts.tolerance = 1e-9;
    opts.max_iterations = 500;

    int converged = 0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::VectorXd q_true(3), q_seed(3);
        for (int i = 0; i < 3; ++i) {
            q_true[i] = angle(rng);
            q_seed[i] = wrapAngle(q_true[i] + 0.4 * angle(rng) / M_PI);
        }
        const Eigen::Vector2d target = eePosition(model, q_true);

        const DlsResult r = dampedLeastSquaresIk(model, target, q_seed, opts);
        if (r.converged) {
            ++converged;
            CHECK((eePosition(model, r.q) - target).norm() < 1e-8);
        }
        CHECK(r.q.allFinite());
    }
    CHECK(converged > 190);
}

TEST_CASE("damped least squares agrees with the closed form", "[ik][dls]") {
    const ArmModel model = make2R();
    const Eigen::Vector2d target(0.8, 0.5);

    const AnalyticIkResult exact = analyticIk2R(model, target);
    REQUIRE(exact.size() == 2);

    DlsOptions opts;
    opts.tolerance = 1e-10;
    opts.max_iterations = 500;

    for (const Eigen::VectorXd& branch : exact.solutions) {
        // Seeded near a branch, the numerical solver should land on it.
        const Eigen::VectorXd seed = wrapAngles(branch + Eigen::VectorXd::Constant(2, 0.1));
        const DlsResult r = dampedLeastSquaresIk(model, target, seed, opts);
        REQUIRE(r.converged);
        CHECK(angleDiff(r.q, branch).norm() == Approx(0.0).margin(1e-6));
    }
}

TEST_CASE("damping keeps the step bounded through a singularity", "[ik][dls]") {
    const ArmModel model = make2R(1.0, 1.0);
    // Straight out and asking for a target beyond the reach: the Jacobian is
    // rank deficient exactly here, which is what blows the pseudoinverse up.
    const Eigen::VectorXd seed = vec({0.0, 0.0});
    const Eigen::Vector2d unreachable(3.0, 0.0);

    DlsOptions opts;
    opts.max_iterations = 200;
    const DlsResult r = dampedLeastSquaresIk(model, unreachable, seed, opts);

    CHECK_FALSE(r.converged);  // it cannot be reached
    CHECK(r.q.allFinite());
    CHECK(r.sigma_min < 1e-6);
    // It should stall at full extension rather than fly off.
    CHECK(eePosition(model, r.q).norm() == Approx(2.0).margin(1e-3));

    SECTION("a target just inside the boundary is still solved") {
        const DlsResult near = dampedLeastSquaresIk(model, Eigen::Vector2d(1.999, 0.0), seed, opts);
        CHECK(near.q.allFinite());
        CHECK((eePosition(model, near.q) - Eigen::Vector2d(1.999, 0.0)).norm() < 1e-3);
    }
}

TEST_CASE("damped least squares can be held inside the joint limits", "[ik][dls]") {
    ArmModel model = make3R();
    model.joint_min << -0.5, -0.5, -0.5;
    model.joint_max << 0.5, 0.5, 0.5;

    DlsOptions opts;
    opts.respect_limits = true;
    opts.max_iterations = 300;

    const DlsResult r = dampedLeastSquaresIk(model, Eigen::Vector2d(0.5, 1.5), vec({0.0, 0.0, 0.0}),
                                             opts);
    CHECK(model.withinLimits(r.q, 1e-12));
}

TEST_CASE("damped least squares can solve for orientation too", "[ik][dls]") {
    const ArmModel model = make3R();
    const Eigen::VectorXd q_true = vec({0.5, -0.6, 0.7});
    const FkResult fk = forwardKinematics(model, q_true);

    DlsOptions opts;
    opts.solve_orientation = true;
    opts.target_orientation = fk.eeOrientation();
    opts.tolerance = 1e-9;
    opts.max_iterations = 500;

    const DlsResult r = dampedLeastSquaresIk(model, fk.eePosition(), vec({0.4, -0.5, 0.6}), opts);
    REQUIRE(r.converged);
    const FkResult got = forwardKinematics(model, r.q);
    CHECK((got.eePosition() - fk.eePosition()).norm() < 1e-8);
    CHECK(std::abs(angleDiff(got.eeOrientation(), fk.eeOrientation())) < 1e-8);
}
