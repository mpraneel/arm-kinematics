#pragma once

#include <SFML/Graphics.hpp>

#include <optional>
#include <string>

#include "arm/supervisor.hpp"
#include "scene.hpp"

namespace viz {

/// Metres to pixels, with y pointing up in the world and down on screen.
struct Camera {
    sf::Vector2f origin_px{470.f, 400.f};
    float px_per_m = 190.f;

    sf::Vector2f toScreen(const Eigen::Vector2d& p) const;
    Eigen::Vector2d toWorld(sf::Vector2f p) const;
    float toPixels(double metres) const { return static_cast<float>(metres) * px_per_m; }
};

struct Palette {
    sf::Color background{18, 20, 26};
    sf::Color panel{26, 29, 37};
    sf::Color grid{34, 38, 48};
    sf::Color link{120, 170, 255};
    sf::Color link_bad{240, 96, 96};
    sf::Color joint{225, 232, 245};
    sf::Color ghost_joint{110, 220, 160};
    sf::Color ghost_cart{245, 175, 80};
    sf::Color obstacle{90, 70, 90};
    sf::Color obstacle_edge{170, 110, 150};
    sf::Color target{255, 214, 92};
    sf::Color ellipsoid{150, 120, 255};
    sf::Color text{215, 222, 235};
    sf::Color text_dim{130, 140, 158};
    sf::Color accent{255, 120, 120};
};

/// A system font, if one can be found. Text is skipped when it cannot.
std::optional<sf::Font> loadFont();

void drawWorld(sf::RenderTarget& t, const Camera& cam, const arm::World& world, const Palette& pal);
void drawWorkspaceBounds(sf::RenderTarget& t, const Camera& cam, const arm::ArmModel& model,
                         const Palette& pal);
void drawArm(sf::RenderTarget& t, const Camera& cam, const arm::ArmModel& model,
             const Eigen::VectorXd& q, sf::Color color, float alpha = 1.f);
void drawTarget(sf::RenderTarget& t, const Camera& cam, const Eigen::Vector2d& p, sf::Color color);
void drawTrail(sf::RenderTarget& t, const Camera& cam, const std::vector<Eigen::Vector2d>& pts,
               sf::Color color);
/// The manipulability ellipsoid at the end effector: unit joint velocity in,
/// this ellipse of end effector velocity out.
void drawEllipsoid(sf::RenderTarget& t, const Camera& cam, const Eigen::Vector2d& at,
                   const Eigen::MatrixXd& axes, sf::Color color, double scale = 0.35);
/// Condition number over time, log scaled.
void drawConditionPlot(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene,
                       const Palette& pal, const sf::Font* font);
/// theta1 against theta2 for a 2R arm, with the C-space obstacles shaded.
void drawJointSpace(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene,
                    const Palette& pal, const sf::Font* font);
void drawHud(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene, const Palette& pal,
             const sf::Font* font);

/// One complete frame, used by both the interactive app and the screenshot mode.
void drawFrame(sf::RenderTarget& t, const Scene& scene, const Camera& cam, const Palette& pal,
               const sf::Font* font);

}  // namespace viz
