#include <SFML/Graphics.hpp>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "render.hpp"
#include "scene.hpp"

namespace {

/// Two nearly extended poses: the straight line between them in the workspace
/// hugs the edge of the reachable set, which is where the two interpolations
/// come apart.
void loadDemoPoses(viz::Scene& scene) {
    const std::size_t n = scene.model.dof();
    Eigen::VectorXd a = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
    a[0] = 0.95;
    b[0] = -0.95;
    if (n > 1) {
        a[1] = -0.25;
        b[1] = 0.25;
    }
    if (n > 2) {
        a[2] = -0.2;
        b[2] = 0.2;
    }
    scene.q = a;
    scene.captureA();
    scene.q = b;
    scene.captureB();
    scene.q = a;
    scene.playDemo();
}

viz::Camera cameraFor(const viz::Scene& scene, int width, int height) {
    viz::Camera cam;
    cam.origin_px = {(width - 320.f) * 0.5f, height * 0.5f};
    cam.px_per_m = std::min((width - 320.f), static_cast<float>(height)) /
                   (2.6f * static_cast<float>(scene.model.reach()));
    return cam;
}

/// Builds the scene an offscreen capture should show.
viz::Scene buildCaptureScene(const std::string& view, int dof, double demo_t, bool supervisor) {
    viz::Scene scene;
    scene.setDof(dof);
    scene.view = view == "joint" ? viz::View::JointSpace : viz::View::Workspace;
    scene.setTarget(Eigen::Vector2d(1.35, 0.75));

    if (supervisor) {
        scene.startSupervisorPlayback();
        return scene;
    }
    if (demo_t > 0.0) {
        loadDemoPoses(scene);
        // Step at a fixed rate so the capture is deterministic.
        const double dt = 1.0 / 120.0;
        while (scene.demo.playing && scene.demo.t < demo_t) scene.stepDemo(dt);
        scene.stopDemo();
    }
    // Fill the plot history without advancing anything any further.
    for (int i = 0; i < 40; ++i) scene.update(1.0 / 60.0);
    return scene;
}

int runScreenshot(const std::string& path, int width, int height, const std::string& view, int dof,
                  double demo_t, bool supervisor) {
    viz::Scene scene = buildCaptureScene(view, dof, demo_t, supervisor);
    sf::RenderTexture texture(sf::Vector2u{static_cast<unsigned>(width),
                                           static_cast<unsigned>(height)});
    const std::optional<sf::Font> font = viz::loadFont();

    viz::drawFrame(texture, scene, cameraFor(scene, width, height), viz::Palette{},
                   font ? &*font : nullptr);
    texture.display();

    if (!texture.getTexture().copyToImage().saveToFile(path)) {
        std::cerr << "failed to write " << path << "\n";
        return 1;
    }
    std::cout << "wrote " << path << "\n";
    return 0;
}

/// A numbered PNG sequence, for assembling a recording without a window.
int runFrames(const std::string& dir, int count, double frame_dt, int width, int height,
              const std::string& view, int dof, bool supervisor) {
    viz::Scene scene = buildCaptureScene(view, dof, supervisor ? 0.0 : 1e-9, supervisor);
    if (!supervisor) {
        loadDemoPoses(scene);  // restart the interpolation from the top
    }

    sf::RenderTexture texture(sf::Vector2u{static_cast<unsigned>(width),
                                           static_cast<unsigned>(height)});
    const std::optional<sf::Font> font = viz::loadFont();
    const viz::Camera cam = cameraFor(scene, width, height);

    for (int i = 0; i < count; ++i) {
        scene.update(frame_dt);
        viz::drawFrame(texture, scene, cam, viz::Palette{}, font ? &*font : nullptr);
        texture.display();

        std::ostringstream name;
        name << dir << "/frame_" << std::setw(4) << std::setfill('0') << i << ".png";
        if (!texture.getTexture().copyToImage().saveToFile(name.str())) {
            std::cerr << "failed to write " << name.str() << "\n";
            return 1;
        }
    }
    std::cout << "wrote " << count << " frames to " << dir << "\n";
    return 0;
}

int runInteractive(int width, int height, int dof, bool supervisor) {
    sf::RenderWindow window(sf::VideoMode({static_cast<unsigned>(width),
                                           static_cast<unsigned>(height)}),
                            "arm-kinematics");
    window.setFramerateLimit(60);

    viz::Scene scene;
    scene.setDof(dof);
    if (supervisor) scene.startSupervisorPlayback();
    const std::optional<sf::Font> font = viz::loadFont();
    const viz::Palette pal;

    viz::Camera cam;
    cam.origin_px = {(width - 320.f) * 0.5f, height * 0.5f};
    cam.px_per_m = std::min((width - 320.f), static_cast<float>(height)) /
                   (2.6f * static_cast<float>(scene.model.reach()));

    bool dragging = false;
    sf::Clock clock;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                using K = sf::Keyboard::Key;
                switch (key->code) {
                    case K::Escape: window.close(); break;
                    case K::A: scene.captureA(); break;
                    case K::B: scene.captureB(); break;
                    case K::Space:
                        if (scene.demo.playing) scene.stopDemo();
                        else scene.playDemo();
                        break;
                    case K::P: loadDemoPoses(scene); break;
                    case K::S:
                        if (scene.playback.active) scene.stopSupervisorPlayback();
                        else scene.startSupervisorPlayback();
                        break;
                    case K::J:
                        scene.view = scene.view == viz::View::Workspace ? viz::View::JointSpace
                                                                        : viz::View::Workspace;
                        if (scene.view == viz::View::JointSpace) scene.rebuildCSpace();
                        break;
                    case K::E: scene.show_ellipsoid = !scene.show_ellipsoid; break;
                    case K::Num2: scene.setDof(2); break;
                    case K::Num3: scene.setDof(3); break;
                    case K::R: scene.setDof(static_cast<int>(scene.model.dof())); break;
                    default: break;
                }
            } else if (const auto* mb = event->getIf<sf::Event::MouseButtonPressed>()) {
                const Eigen::Vector2d world = cam.toWorld(sf::Vector2f(mb->position));
                if (mb->button == sf::Mouse::Button::Left) {
                    dragging = true;
                    scene.setTarget(world);
                } else if (mb->button == sf::Mouse::Button::Right) {
                    scene.world.circles.push_back(arm::Circle{world, 0.2});
                    scene.rebuildCSpace();
                    scene.solveIk();
                }
            } else if (event->is<sf::Event::MouseButtonReleased>()) {
                dragging = false;
            } else if (const auto* mm = event->getIf<sf::Event::MouseMoved>()) {
                if (dragging) scene.setTarget(cam.toWorld(sf::Vector2f(mm->position)));
            }
        }

        scene.update(clock.restart().asSeconds());
        viz::drawFrame(window, scene, cam, pal, font ? &*font : nullptr);
        window.display();
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::string screenshot;
    std::string frames_dir;
    int frame_count = 120;
    double frame_dt = 1.0 / 50.0;
    bool supervisor = false;
    std::string view = "workspace";
    int width = 1280;
    int height = 800;
    int dof = 3;
    double demo_t = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : std::string(); };
        if (arg == "--screenshot") screenshot = next();
        else if (arg == "--view") view = next();
        else if (arg == "--dof") dof = std::stoi(next());
        else if (arg == "--demo") demo_t = std::stod(next());
        else if (arg == "--width") width = std::stoi(next());
        else if (arg == "--height") height = std::stoi(next());
        else if (arg == "--frames") frames_dir = next();
        else if (arg == "--frame-count") frame_count = std::stoi(next());
        else if (arg == "--frame-dt") frame_dt = std::stod(next());
        else if (arg == "--supervisor") supervisor = true;
        else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: arm_viz [--screenshot out.png] [--view workspace|joint]\n"
                      << "               [--dof N] [--demo T] [--supervisor]\n"
                      << "               [--frames DIR [--frame-count N] [--frame-dt S]]\n"
                      << "               [--width W] [--height H]\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            return 2;
        }
    }

    if (!frames_dir.empty()) {
        return runFrames(frames_dir, frame_count, frame_dt, width, height, view, dof, supervisor);
    }
    if (!screenshot.empty()) {
        return runScreenshot(screenshot, width, height, view, dof, demo_t, supervisor);
    }
    return runInteractive(width, height, dof, supervisor);
}
