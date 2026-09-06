// Measurement harness: runs the same seeded command sequence with the
// supervisor off and on, and reports what the guardrail costs and what it buys.
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "arm/scenario.hpp"

namespace {

struct Run {
    std::string name;
    arm::RunMetrics metrics;
    /// Per check: how often it fired at all, and how often it rejected. A
    /// check can fire without rejecting - a branch switch or a rate clamp
    /// modifies the command and lets it through.
    std::map<std::string, int> fired;
    std::map<std::string, int> rejections;
};

Run execute(const std::string& name, bool enabled, arm::Fallback fallback, int commands,
            std::uint64_t seed, double dt, const std::string& log_path) {
    const arm::ArmModel model = arm::scenario::model();
    const arm::World world = arm::scenario::world();

    arm::ScriptedController controller(model, world, arm::scenario::controller(seed, dt));
    arm::Supervisor supervisor(model, world,
                               arm::scenario::supervisor(enabled, fallback, seed, dt));

    supervisor.setConfiguration(
        arm::scenario::startConfiguration(model, controller.nominalAt(0.0)));

    std::vector<arm::CommandRecord> records;
    records.reserve(static_cast<std::size_t>(commands));

    std::ofstream log;
    if (!log_path.empty()) log.open(log_path);

    for (int i = 0; i < commands; ++i) {
        const arm::Command cmd = controller.next();
        const arm::CommandRecord rec = supervisor.step(cmd);
        if (log.is_open()) log << rec.toJson() << "\n";
        records.push_back(rec);
    }

    Run run;
    run.name = name;
    run.metrics = arm::RunMetrics::fromRecords(records);
    for (const arm::CommandRecord& r : records) {
        if (r.failed_check != arm::Check::None) ++run.rejections[arm::checkName(r.failed_check)];
        for (const arm::Check c : {arm::Check::Reachability, arm::Check::JointLimits,
                                   arm::Check::VelocityLimits, arm::Check::Singularity,
                                   arm::Check::Collision}) {
            if (r.triggered_checks & arm::checkBit(c)) ++run.fired[arm::checkName(c)];
        }
    }
    return run;
}

std::string num(double v, int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

std::string pct(double fraction) { return num(100.0 * fraction, 1) + "%"; }

std::string table(const std::vector<Run>& runs) {
    std::ostringstream os;
    os << "| Metric |";
    for (const Run& r : runs) os << " " << r.name << " |";
    os << "\n|---|";
    for (std::size_t i = 0; i < runs.size(); ++i) os << "---|";
    os << "\n";

    const auto row = [&](const char* label, auto&& value) {
        os << "| " << label << " |";
        for (const Run& r : runs) os << " " << value(r) << " |";
        os << "\n";
    };

    row("Joint limit violations",
        [](const Run& r) { return std::to_string(r.metrics.joint_limit_violations); });
    row("Velocity limit violations",
        [](const Run& r) { return std::to_string(r.metrics.velocity_violations); });
    row("Collisions", [](const Run& r) { return std::to_string(r.metrics.collisions); });
    row("Targets reached", [](const Run& r) {
        return std::to_string(r.metrics.targets_reached) + " / " +
               std::to_string(r.metrics.commands);
    });
    row("Clean targets reached", [](const Run& r) {
        return std::to_string(r.metrics.clean_reached) + " / " +
               std::to_string(r.metrics.clean_commands);
    });
    row("Intervention rate", [](const Run& r) { return pct(r.metrics.interventionRate()); });
    row("p50 supervisor latency", [](const Run& r) { return num(r.metrics.p50_latency_us, 2) + " us"; });
    row("p99 supervisor latency", [](const Run& r) { return num(r.metrics.p99_latency_us, 2) + " us"; });
    return os.str();
}

std::string checkTable(const std::vector<Run>& runs) {
    std::ostringstream os;
    os << "| Check |";
    for (const Run& r : runs) os << " " << r.name << " |";
    os << "\n|---|";
    for (std::size_t i = 0; i < runs.size(); ++i) os << "---|";
    os << "\n";

    const auto count = [](const std::map<std::string, int>& m, const char* key) {
        const auto it = m.find(key);
        return it == m.end() ? 0 : it->second;
    };
    for (const char* check : {"reachability", "joint_limits", "velocity_limits", "singularity",
                              "collision"}) {
        os << "| " << check << " |";
        for (const Run& r : runs) {
            os << " " << count(r.fired, check) << " fired / " << count(r.rejections, check)
               << " rejected |";
        }
        os << "\n";
    }
    return os.str();
}

}  // namespace

int main(int argc, char** argv) {
    int commands = 2000;
    std::uint64_t seed = 42;
    double dt = 0.02;
    std::string log_dir = "bench_logs";
    std::string markdown;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
        if (arg == "--commands") commands = std::stoi(next());
        else if (arg == "--seed") seed = std::stoull(next());
        else if (arg == "--dt") dt = std::stod(next());
        else if (arg == "--log-dir") log_dir = next();
        else if (arg == "--markdown") markdown = next();
        else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: arm_bench [--commands N] [--seed S] [--dt SECONDS]\n"
                      << "                 [--log-dir DIR] [--markdown OUT.md]\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            return 2;
        }
    }

    const auto log_path = [&](const char* name) {
        return log_dir.empty() ? std::string() : log_dir + "/" + name + ".jsonl";
    };

    std::vector<Run> runs;
    runs.push_back(execute("Supervisor off", false, arm::Fallback::Hold, commands, seed, dt,
                           log_path("supervisor_off")));
    runs.push_back(execute("On, hold", true, arm::Fallback::Hold, commands, seed, dt,
                           log_path("supervisor_hold")));
    runs.push_back(execute("On, clamp", true, arm::Fallback::Clamp, commands, seed, dt,
                           log_path("supervisor_clamp")));
    runs.push_back(execute("On, plan", true, arm::Fallback::Plan, commands, seed, dt,
                           log_path("supervisor_plan")));

    std::ostringstream report;
    report << "Seeded run: " << commands << " commands, seed " << seed << ", dt " << dt << " s.\n\n"
           << table(runs) << "\nChecks:\n\n"
           << checkTable(runs);

    std::cout << report.str();
    if (!markdown.empty()) {
        std::ofstream out(markdown);
        out << report.str();
        std::cout << "\nwrote " << markdown << "\n";
    }
    return 0;
}
