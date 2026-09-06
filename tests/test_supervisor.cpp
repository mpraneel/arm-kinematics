#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "arm/scenario.hpp"

using Catch::Approx;
using namespace arm;

namespace {

ArmModel make3R() { return scenario::model(); }

World makeWorld() { return scenario::world(); }

SupervisorConfig makeConfig(Fallback fallback = Fallback::Clamp) {
    return scenario::supervisor(true, fallback);
}

ControllerConfig makeControllerConfig(std::uint64_t seed = 42) {
    return scenario::controller(seed);
}

Eigen::VectorXd home() {
    Eigen::VectorXd q(3);
    q << 0.6, -1.0, 0.7;
    return q;
}

/// Start the arm already on the controller's nominal path, so a policy that
/// refuses large steps is not judged on a jump it never should have been asked
/// to make in the first place.
Eigen::VectorXd startOnPath(const ArmModel& model, const ScriptedController& controller) {
    return scenario::startConfiguration(model, controller.nominalAt(0.0));
}

Command makeCommand(const Eigen::Vector2d& target, int index = 0) {
    Command c;
    c.index = index;
    c.target = target;
    return c;
}

std::vector<CommandRecord> run(bool enabled, Fallback fallback, int steps,
                               std::uint64_t seed = 42) {
    const ArmModel model = make3R();
    const World world = makeWorld();
    SupervisorConfig cfg = makeConfig(fallback);
    cfg.enabled = enabled;
    cfg.rrt.seed = seed;

    ScriptedController controller(model, world, makeControllerConfig(seed));
    Supervisor supervisor(model, world, cfg);
    supervisor.setConfiguration(startOnPath(model, controller));

    std::vector<CommandRecord> records;
    for (int i = 0; i < steps; ++i) records.push_back(supervisor.step(controller.next()));
    return records;
}

}  // namespace

TEST_CASE("the scripted controller is reproducible and labels its own faults", "[controller]") {
    const ArmModel model = make3R();
    const World world = makeWorld();

    ScriptedController a(model, world, makeControllerConfig(7));
    ScriptedController b(model, world, makeControllerConfig(7));
    ScriptedController c(model, world, makeControllerConfig(8));

    bool saw_difference = false;
    int faulted = 0;
    for (int i = 0; i < 400; ++i) {
        const Command x = a.next();
        const Command y = b.next();
        const Command z = c.next();
        REQUIRE((x.target - y.target).norm() == Approx(0.0).margin(1e-15));
        REQUIRE(x.faults == y.faults);
        if ((x.target - z.target).norm() > 1e-9) saw_difference = true;
        if (x.faults & ~kFaultNoise) ++faulted;
    }
    CHECK(saw_difference);       // a different seed is a different sequence
    CHECK(faulted > 0);          // and the failure modes actually fire
    CHECK(faultNames(kFaultStale | kFaultSingular) == "stale|singular");
    CHECK(faultNames(kFaultNone) == "none");
}

TEST_CASE("a clean command is forwarded untouched", "[supervisor]") {
    Supervisor sup(make3R(), makeWorld(), makeConfig());
    sup.setConfiguration(home());

    // A target a short step away from where the arm already is.
    const Eigen::Vector2d here = eePosition(make3R(), home());
    const CommandRecord rec = sup.step(makeCommand(here + Eigen::Vector2d(0.01, 0.005)));

    CHECK(rec.failed_check == Check::None);
    CHECK_FALSE(rec.intervened);
    CHECK_FALSE(rec.fallback_engaged);
    CHECK(rec.reached_target);
    CHECK(rec.latency_us > 0.0);
}

TEST_CASE("an unreachable target is rejected by the cheapest check", "[supervisor]") {
    Supervisor sup(make3R(), makeWorld(), makeConfig(Fallback::Hold));
    sup.setConfiguration(home());

    const CommandRecord rec = sup.step(makeCommand(Eigen::Vector2d(4.0, 0.0)));
    CHECK(rec.failed_check == Check::Reachability);
    CHECK(rec.intervened);
    CHECK(rec.fallback_engaged);
    CHECK(angleDiff(rec.q_after, home()).norm() == Approx(0.0).margin(1e-12));
    CHECK_FALSE(rec.reached_target);
}

TEST_CASE("a target inside an obstacle never gets forwarded", "[supervisor]") {
    Supervisor sup(make3R(), makeWorld(), makeConfig(Fallback::Hold));
    sup.setConfiguration(home());

    const CommandRecord rec = sup.step(makeCommand(makeWorld().circles[0].center));
    CHECK(rec.failed_check == Check::Reachability);
    CHECK_FALSE(rec.collision);
}

TEST_CASE("velocity limits are clamped or rejected by policy", "[supervisor]") {
    const ArmModel model = make3R();
    // Reachable, legal, and far enough that tracking it in one step would need
    // an absurd joint rate.
    const Eigen::Vector2d far_side(0.4, -1.6);

    SECTION("clamp scales the step down and keeps moving") {
        Supervisor sup(model, makeWorld(), makeConfig(Fallback::Clamp));
        sup.setConfiguration(home());
        const CommandRecord rec = sup.step(makeCommand(far_side));

        CHECK(rec.intervened);
        CHECK_FALSE(rec.velocity_violation);
        const Eigen::VectorXd rate = angleDiff(rec.q_after, rec.q_before).cwiseAbs() / 0.02;
        CHECK(rate.maxCoeff() <= 3.0 + 1e-9);
        CHECK(angleDiff(rec.q_after, rec.q_before).norm() > 1e-9);  // it did move
    }
    SECTION("with clamping disabled the command is refused outright") {
        SupervisorConfig cfg = makeConfig(Fallback::Hold);
        cfg.clamp_velocity = false;
        Supervisor sup(model, makeWorld(), cfg);
        sup.setConfiguration(home());
        const CommandRecord rec = sup.step(makeCommand(far_side));

        CHECK(rec.failed_check == Check::VelocityLimits);
        CHECK(angleDiff(rec.q_after, rec.q_before).norm() == Approx(0.0).margin(1e-12));
    }
}

TEST_CASE("a near singular target is refused rather than recovered from", "[supervisor]") {
    const ArmModel model = make3R();
    Supervisor sup(model, World{}, makeConfig(Fallback::Hold));

    // Almost straight out, so full extension is one small step away and the
    // velocity check will not fire first.
    Eigen::VectorXd nearly_straight(3);
    nearly_straight << 0.0, 0.02, 0.02;
    sup.setConfiguration(nearly_straight);

    const CommandRecord rec = sup.step(makeCommand(Eigen::Vector2d(model.reach(), 0.0)));
    CHECK(rec.failed_check == Check::Singularity);
    CHECK(rec.sigma_min < 1.0);
}

TEST_CASE("all IK branches are checked before rejecting on joint limits", "[supervisor]") {
    ArmModel model = make3R();
    // Only negative elbow angles are legal, so the branch the numerical solver
    // walks into is illegal while another one is fine.
    model.joint_min << -3.1, -2.5, -3.1;
    model.joint_max << 3.1, 0.0, 3.1;

    Supervisor sup(model, World{}, makeConfig(Fallback::Clamp));
    Eigen::VectorXd start(3);
    start << 0.4, -0.6, 0.3;
    sup.setConfiguration(start);

    // Sweep a short arc; nothing should ever leave the limits.
    for (int i = 0; i < 200; ++i) {
        const double a = 0.4 + 0.004 * i;
        const Eigen::Vector2d target(1.6 * std::cos(a), 1.6 * std::sin(a));
        const CommandRecord rec = sup.step(makeCommand(target, i));
        REQUIRE(model.withinLimits(rec.q_after, 1e-9));
        REQUIRE_FALSE(rec.limit_violation);
    }
}

TEST_CASE("with the supervisor on, nothing the arm does is illegal", "[supervisor][run]") {
    for (const Fallback fallback : {Fallback::Hold, Fallback::Clamp, Fallback::Plan}) {
        const std::vector<CommandRecord> records = run(true, fallback, 400);
        const RunMetrics m = RunMetrics::fromRecords(records);

        INFO("fallback " << fallbackName(fallback));
        CHECK(m.joint_limit_violations == 0);
        CHECK(m.velocity_violations == 0);
        CHECK(m.collisions == 0);
        // It intervenes on the bad commands, not on all of them.
        CHECK(m.interventions > 0);
        CHECK(m.interventionRate() < 1.0);
        CHECK(m.p50_latency_us > 0.0);
        CHECK(m.p99_latency_us >= m.p50_latency_us);
    }
}

TEST_CASE("with the supervisor off, the same commands break things", "[supervisor][run]") {
    const RunMetrics off = RunMetrics::fromRecords(run(false, Fallback::Hold, 400));
    const RunMetrics on = RunMetrics::fromRecords(run(true, Fallback::Clamp, 400));

    // This comparison is the point of the whole layer.
    CHECK(off.joint_limit_violations > 0);
    CHECK(off.velocity_violations > 0);
    CHECK(off.collisions > 0);
    CHECK(off.interventions == 0);
    CHECK(off.p50_latency_us == Approx(0.0));

    CHECK(on.joint_limit_violations < off.joint_limit_violations);
    CHECK(on.velocity_violations < off.velocity_violations);
    CHECK(on.collisions < off.collisions);
}

TEST_CASE("a supervised run is reproducible", "[supervisor][run]") {
    const std::vector<CommandRecord> a = run(true, Fallback::Clamp, 200);
    const std::vector<CommandRecord> b = run(true, Fallback::Clamp, 200);
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        REQUIRE((a[i].q_after - b[i].q_after).norm() == Approx(0.0).margin(1e-15));
        REQUIRE(a[i].failed_check == b[i].failed_check);
    }
}

TEST_CASE("every command produces one structured log line", "[supervisor][log]") {
    const std::vector<CommandRecord> records = run(true, Fallback::Clamp, 20);
    for (const CommandRecord& r : records) {
        const std::string json = r.toJson();
        CHECK(json.front() == '{');
        CHECK(json.back() == '}');
        CHECK(json.find("\"latency_us\"") != std::string::npos);
        CHECK(json.find("\"failed_check\"") != std::string::npos);
        CHECK(json.find('\n') == std::string::npos);  // one line per command
    }
}
