#include "scene.hpp"

#include <algorithm>
#include <cmath>

namespace viz {
namespace {

arm::DlsOptions liveIkOptions() {
    arm::DlsOptions opts;
    opts.max_iterations = 60;
    opts.tolerance = 1e-6;
    opts.max_step = 0.25;
    return opts;
}

}  // namespace

Scene::Scene() {
    setDof(3);
}

void Scene::setDof(int n) {
    Eigen::VectorXd lengths(n);
    if (n == 2) {
        lengths << 1.0, 0.9;
    } else if (n == 3) {
        lengths << 1.0, 0.8, 0.6;
    } else {
        lengths.setConstant(2.4 / n);
    }
    model = arm::ArmModel(lengths, 2.9);
    model.link_radius = 0.04;

    q = Eigen::VectorXd::Zero(n);
    q[0] = 0.5;
    if (n > 1) q[1] = -0.8;
    if (n > 2) q[2] = 0.6;

    demo = DualInterp{};
    cond_history.clear();
    resetWorld();
    solveIk();
    rebuildCSpace();
}

void Scene::resetWorld() {
    world = arm::World{};
    world.circles.push_back(arm::Circle{Eigen::Vector2d(-0.55, 1.05), 0.28});
    world.circles.push_back(arm::Circle{Eigen::Vector2d(1.15, -0.95), 0.22});
    world.boxes.push_back(arm::Aabb{Eigen::Vector2d(0.55, 1.25), Eigen::Vector2d(1.45, 1.55)});
}

void Scene::setTarget(const Eigen::Vector2d& t) {
    target = t;
    solveIk();
}

void Scene::solveIk() {
    const double r = (target - model.base.translation()).norm();
    target_reachable = r <= model.reach() && r >= model.innerReach();

    const arm::DlsResult res = arm::dampedLeastSquaresIk(model, target, q, liveIkOptions());
    q = res.q;
    ik_converged = res.converged;
    ik_error = res.error;
    collision = arm::checkCollision(model, q, world);
}

arm::Manipulability Scene::manipulability() const {
    return arm::manipulability(model, q);
}

void Scene::update(double dt) {
    stepDemo(dt);

    const arm::Manipulability m = manipulability();
    cond_history.push_back(m.condition_number);
    while (cond_history.size() > cond_history_max) cond_history.pop_front();
}

void Scene::captureA() {
    demo.qa = q;
    demo.has_a = true;
    demo.t = 0.0;
    demo.trail_joint.clear();
    demo.trail_cart.clear();
}

void Scene::captureB() {
    demo.qb = q;
    demo.has_b = true;
    demo.t = 0.0;
    demo.trail_joint.clear();
    demo.trail_cart.clear();
}

void Scene::playDemo() {
    if (!demo.ready()) return;
    demo.playing = true;
    demo.t = 0.0;
    demo.cart_ok = true;
    demo.q_joint = demo.qa;
    demo.q_cart = demo.qa;
    demo.trail_joint.clear();
    demo.trail_cart.clear();
}

void Scene::stopDemo() {
    demo.playing = false;
}

void Scene::stepDemo(double dt) {
    if (!demo.playing || !demo.ready()) return;

    demo.t = std::min(1.0, demo.t + demo.speed * dt);

    // Joint space: straight line in q, along the shortest wrapped path.
    demo.q_joint = arm::wrapAngles(demo.qa + demo.t * arm::angleDiff(demo.qb, demo.qa));

    // Cartesian: straight line in the workspace, IK solved per step from the
    // previous configuration so the ghost tracks continuously.
    const Eigen::Vector2d pa = arm::eePosition(model, demo.qa);
    const Eigen::Vector2d pb = arm::eePosition(model, demo.qb);
    const Eigen::Vector2d p = pa + demo.t * (pb - pa);

    arm::DlsOptions opts = liveIkOptions();
    opts.max_iterations = 120;
    const arm::DlsResult res = arm::dampedLeastSquaresIk(model, p, demo.q_cart, opts);
    demo.q_cart = res.q;
    demo.cart_error = (arm::eePosition(model, demo.q_cart) - p).norm();
    // Once it falls behind the commanded point it has hit a singularity or the
    // edge of the workspace, and it never silently recovers its claim.
    if (demo.cart_error > 1e-3) demo.cart_ok = false;

    demo.trail_joint.push_back(arm::eePosition(model, demo.q_joint));
    demo.trail_cart.push_back(arm::eePosition(model, demo.q_cart));

    if (demo.t >= 1.0) demo.playing = false;
}

void Scene::rebuildCSpace(int resolution) {
    cspace.clear();
    cspace_res = 0;
    if (model.dof() != 2) return;

    cspace_res = resolution;
    cspace.assign(static_cast<std::size_t>(resolution) * resolution, 0);

    Eigen::VectorXd c(2);
    for (int i = 0; i < resolution; ++i) {
        c[0] = -M_PI + 2.0 * M_PI * (i + 0.5) / resolution;
        for (int j = 0; j < resolution; ++j) {
            c[1] = -M_PI + 2.0 * M_PI * (j + 0.5) / resolution;
            unsigned char flag = 0;
            if (arm::checkCollision(model, c, world).hit()) flag = 1;
            else if (!model.withinLimits(c)) flag = 2;
            cspace[static_cast<std::size_t>(i) * resolution + j] = flag;
        }
    }
}

}  // namespace viz
