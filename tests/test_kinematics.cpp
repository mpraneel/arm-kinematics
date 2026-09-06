#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <random>

#include "arm/kinematics.hpp"

using Catch::Approx;
using namespace arm;

namespace {

ArmModel make3R(double l1 = 1.0, double l2 = 0.8, double l3 = 0.6) {
    Eigen::VectorXd lengths(3);
    lengths << l1, l2, l3;
    return ArmModel(lengths);
}

Eigen::VectorXd vec(std::initializer_list<double> v) {
    Eigen::VectorXd q(static_cast<Eigen::Index>(v.size()));
    Eigen::Index i = 0;
    for (double x : v) q[i++] = x;
    return q;
}

}  // namespace

TEST_CASE("link transform rotates then translates along the new x axis", "[fk]") {
    const Eigen::Isometry2d t = linkTransform(M_PI / 2.0, 2.0);
    CHECK(t.translation().x() == Approx(0.0).margin(1e-12));
    CHECK(t.translation().y() == Approx(2.0));
    CHECK(Eigen::Rotation2Dd(t.linear()).angle() == Approx(M_PI / 2.0));
}

TEST_CASE("forward kinematics places the end effector where the geometry says", "[fk]") {
    Eigen::VectorXd lengths(2);
    lengths << 1.0, 1.0;
    const ArmModel model(lengths);

    SECTION("straight out") {
        const Eigen::Vector2d p = eePosition(model, vec({0.0, 0.0}));
        CHECK(p.x() == Approx(2.0));
        CHECK(p.y() == Approx(0.0).margin(1e-12));
    }
    SECTION("shoulder up") {
        const Eigen::Vector2d p = eePosition(model, vec({M_PI / 2.0, 0.0}));
        CHECK(p.x() == Approx(0.0).margin(1e-12));
        CHECK(p.y() == Approx(2.0));
    }
    SECTION("elbow bent, angles are relative to the previous link") {
        const Eigen::Vector2d p = eePosition(model, vec({0.0, M_PI / 2.0}));
        CHECK(p.x() == Approx(1.0));
        CHECK(p.y() == Approx(1.0));
    }
    SECTION("folded back onto the base") {
        const Eigen::Vector2d p = eePosition(model, vec({0.0, M_PI}));
        CHECK(p.norm() == Approx(0.0).margin(1e-12));
    }
}

TEST_CASE("forward kinematics reports every intermediate frame", "[fk]") {
    const ArmModel model = make3R();
    const FkResult fk = forwardKinematics(model, vec({0.3, -0.4, 0.5}));

    REQUIRE(fk.frames.size() == model.dof() + 1);
    CHECK(fk.frames.front().translation().norm() == Approx(0.0).margin(1e-12));
    CHECK(fk.points().size() == model.dof() + 1);
    // Each frame is exactly one link length from the previous one.
    for (std::size_t i = 0; i + 1 < fk.frames.size(); ++i) {
        const double d = (fk.frames[i + 1].translation() - fk.frames[i].translation()).norm();
        CHECK(d == Approx(model.link_lengths[static_cast<Eigen::Index>(i)]));
    }
    CHECK(fk.eeOrientation() == Approx(wrapAngle(0.3 - 0.4 + 0.5)));
}

TEST_CASE("a base pose offsets the whole chain", "[fk]") {
    ArmModel model = make3R();
    model.base.translation() = Eigen::Vector2d(1.0, -2.0);
    model.base.linear() = Eigen::Rotation2Dd(M_PI / 2.0).toRotationMatrix();

    const Eigen::VectorXd q = vec({0.0, 0.0, 0.0});
    const Eigen::Vector2d p = eePosition(model, q);
    CHECK(p.x() == Approx(1.0).margin(1e-12));
    CHECK(p.y() == Approx(-2.0 + model.reach()));
}

TEST_CASE("the analytic Jacobian matches finite differences", "[jacobian]") {
    const ArmModel model = make3R();
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);
    const double h = 1e-6;

    for (int trial = 0; trial < 200; ++trial) {
        Eigen::VectorXd q(3);
        for (int i = 0; i < 3; ++i) q[i] = angle(rng);

        const Eigen::MatrixXd J = jacobian(model, q);
        REQUIRE(J.rows() == 3);
        REQUIRE(J.cols() == 3);

        for (int i = 0; i < 3; ++i) {
            Eigen::VectorXd qp = q, qm = q;
            qp[i] += h;
            qm[i] -= h;
            const Eigen::Vector2d d = (eePosition(model, qp) - eePosition(model, qm)) / (2.0 * h);
            CHECK(J(0, i) == Approx(d.x()).margin(1e-6));
            CHECK(J(1, i) == Approx(d.y()).margin(1e-6));
            // Every revolute joint contributes one to one to the ee heading.
            CHECK(J(2, i) == Approx(1.0));
        }
    }
}

TEST_CASE("manipulability collapses at a singularity", "[manipulability]") {
    Eigen::VectorXd lengths(2);
    lengths << 1.0, 1.0;
    const ArmModel model(lengths);

    SECTION("full extension loses a degree of freedom") {
        const Manipulability m = manipulability(model, vec({0.0, 0.0}));
        CHECK(m.sigma_min == Approx(0.0).margin(1e-12));
        CHECK(m.yoshikawa == Approx(0.0).margin(1e-12));
        CHECK(m.condition_number > 1e12);
    }
    SECTION("a right angle elbow is well conditioned") {
        const Manipulability m = manipulability(model, vec({0.0, M_PI / 2.0}));
        CHECK(m.sigma_min > 0.1);
        CHECK(m.condition_number < 10.0);
        // For a square Jacobian, Yoshikawa's measure is just |det J|.
        const Eigen::MatrixXd J = positionJacobian(model, vec({0.0, M_PI / 2.0}));
        CHECK(m.yoshikawa == Approx(std::abs(J.determinant())));
        CHECK(m.ellipsoid_axes.col(0).norm() == Approx(m.sigma_max));
    }
}

TEST_CASE("joint limits", "[model]") {
    Eigen::VectorXd lengths(2);
    lengths << 1.0, 1.0;
    ArmModel model(lengths);
    model.joint_min << -1.0, -0.5;
    model.joint_max << 1.0, 0.5;

    CHECK(model.withinLimits(vec({0.5, 0.25})));
    CHECK_FALSE(model.withinLimits(vec({0.5, 0.9})));
    CHECK(model.firstLimitViolation(vec({2.0, 0.9})) == 0);
    CHECK(model.firstLimitViolation(vec({0.5, 0.9})) == 1);
    const Eigen::VectorXd clamped = model.clampToLimits(vec({2.0, -0.9}));
    CHECK(clamped[0] == Approx(1.0));
    CHECK(clamped[1] == Approx(-0.5));
}

TEST_CASE("reach describes the annulus", "[model]") {
    Eigen::VectorXd lengths(2);
    lengths << 2.0, 1.0;
    const ArmModel model(lengths);
    CHECK(model.reach() == Approx(3.0));
    CHECK(model.innerReach() == Approx(1.0));

    Eigen::VectorXd equal(2);
    equal << 1.0, 1.0;
    CHECK(ArmModel(equal).innerReach() == Approx(0.0));
}

TEST_CASE("angle wrapping is on the half open interval", "[angles]") {
    CHECK(wrapAngle(0.0) == Approx(0.0));
    CHECK(wrapAngle(M_PI) == Approx(M_PI));
    CHECK(wrapAngle(-M_PI) == Approx(M_PI));
    CHECK(wrapAngle(3.0 * M_PI) == Approx(M_PI));
    CHECK(wrapAngle(2.0 * M_PI + 0.5) == Approx(0.5));
    CHECK(angleDiff(0.1, 2.0 * M_PI - 0.1) == Approx(0.2));
}
