#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "arm/collision.hpp"
#include "arm/controller.hpp"
#include "arm/ik.hpp"
#include "arm/kinematics.hpp"
#include "arm/scenario.hpp"
#include "arm/supervisor.hpp"

namespace viz {

/// Two ghost arms travelling between the same pair of poses: one interpolating
/// in joint space, one in Cartesian space with IK solved at every step. Joint
/// space is smooth in q but curved in the workspace; Cartesian is a straight
/// line in the workspace but can run through a singularity or off the edge of
/// the reachable set.
struct DualInterp {
    bool has_a = false;
    bool has_b = false;
    Eigen::VectorXd qa;
    Eigen::VectorXd qb;

    bool playing = false;
    double t = 0.0;
    double speed = 0.4;  ///< fraction of the path per second

    Eigen::VectorXd q_joint;
    Eigen::VectorXd q_cart;
    /// False once the Cartesian ghost could not reach its interpolated target,
    /// which is the whole point of the demo.
    bool cart_ok = true;
    double cart_error = 0.0;

    std::vector<Eigen::Vector2d> trail_joint;
    std::vector<Eigen::Vector2d> trail_cart;

    bool ready() const { return has_a && has_b; }
};

/// The benchmark scenario played back on screen: the same seeded command
/// stream driving two arms, one behind the supervisor and one not.
struct SupervisorPlayback {
    bool active = false;
    std::optional<arm::ScriptedController> controller;
    std::optional<arm::Supervisor> guarded;
    std::optional<arm::Supervisor> unguarded;

    arm::CommandRecord last;      ///< what the supervisor did with this command
    arm::CommandRecord last_raw;  ///< what the same command did unsupervised
    Eigen::VectorXd q_raw;

    int commands = 0;
    int interventions = 0;
    int rejections = 0;
    int raw_violations = 0;
    /// Real time carried over between frames, so playback runs at the
    /// controller's own rate rather than the frame rate.
    double accumulator = 0.0;
    /// The last few rejections, newest first, for the on screen log.
    std::deque<std::string> events;
};

enum class View { Workspace, JointSpace };

/// Everything the visualizer shows, with no dependency on the renderer.
struct Scene {
    arm::ArmModel model;
    Eigen::VectorXd q;
    Eigen::Vector2d target = Eigen::Vector2d(1.2, 0.6);
    arm::World world;

    View view = View::Workspace;
    bool show_ellipsoid = true;

    /// Result of the last live IK solve.
    bool ik_converged = true;
    double ik_error = 0.0;
    bool target_reachable = true;

    arm::CollisionReport collision;
    DualInterp demo;
    SupervisorPlayback playback;

    /// Condition number over time, newest last, for the live plot.
    std::deque<double> cond_history;
    std::size_t cond_history_max = 320;

    /// Joint space occupancy for a 2R arm: res x res, row major over
    /// (theta1, theta2) in (-pi, pi]. Empty unless the arm is 2R.
    std::vector<unsigned char> cspace;
    int cspace_res = 0;

    Scene();

    /// Rebuild for an n-link arm with sensible defaults. 2 and 3 are the
    /// interesting cases; the C-space view needs 2.
    void setDof(int n);
    void resetWorld();

    /// Move the target and solve IK from the current configuration.
    void setTarget(const Eigen::Vector2d& t);
    void solveIk();

    void update(double dt);

    arm::Manipulability manipulability() const;
    arm::FkResult fk() const { return arm::forwardKinematics(model, q); }

    // Dual interpolation demo.
    void captureA();
    void captureB();
    void playDemo();
    void stopDemo();
    void stepDemo(double dt);

    /// Load the benchmark scenario and start playing it back.
    void startSupervisorPlayback(arm::Fallback fallback = arm::Fallback::Clamp,
                                 std::uint64_t seed = 42);
    void stopSupervisorPlayback();
    /// Advance playback by `dt` seconds of wall clock.
    void stepSupervisorPlayback(double dt);

    /// Recompute the 2R C-space occupancy grid. No-op for other chains.
    void rebuildCSpace(int resolution = 220);
};

}  // namespace viz
