#include "render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace viz {
namespace {

sf::Color withAlpha(sf::Color c, float a) {
    c.a = static_cast<std::uint8_t>(std::clamp(a, 0.f, 1.f) * 255.f);
    return c;
}

void drawThickLine(sf::RenderTarget& t, sf::Vector2f a, sf::Vector2f b, float thickness,
                   sf::Color color) {
    const sf::Vector2f d = b - a;
    const float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-6f) return;
    sf::RectangleShape r({len, thickness});
    r.setOrigin({0.f, thickness * 0.5f});
    r.setPosition(a);
    r.setRotation(sf::radians(std::atan2(d.y, d.x)));
    r.setFillColor(color);
    t.draw(r);
}

void drawDisc(sf::RenderTarget& t, sf::Vector2f c, float radius, sf::Color fill,
              sf::Color outline = sf::Color::Transparent, float outline_width = 0.f) {
    sf::CircleShape s(radius, 40);
    s.setOrigin({radius, radius});
    s.setPosition(c);
    s.setFillColor(fill);
    if (outline_width > 0.f) {
        s.setOutlineColor(outline);
        s.setOutlineThickness(outline_width);
    }
    t.draw(s);
}

void drawRing(sf::RenderTarget& t, sf::Vector2f c, float radius, sf::Color color, float width) {
    drawDisc(t, c, radius, sf::Color::Transparent, color, width);
}

std::string fmt(double v, int precision = 3) {
    std::ostringstream os;
    if (!std::isfinite(v)) return "inf";
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

float drawText(sf::RenderTarget& t, const sf::Font* font, const std::string& s, sf::Vector2f pos,
               unsigned size, sf::Color color) {
    if (!font) return pos.y;
    sf::Text text(*font, s, size);
    text.setPosition(pos);
    text.setFillColor(color);
    t.draw(text);
    return pos.y + size * 1.45f;
}

void drawPanel(sf::RenderTarget& t, const sf::FloatRect& box, sf::Color fill, sf::Color edge) {
    sf::RectangleShape r(box.size);
    r.setPosition(box.position);
    r.setFillColor(fill);
    r.setOutlineColor(edge);
    r.setOutlineThickness(1.f);
    t.draw(r);
}

}  // namespace

sf::Vector2f Camera::toScreen(const Eigen::Vector2d& p) const {
    return {origin_px.x + static_cast<float>(p.x()) * px_per_m,
            origin_px.y - static_cast<float>(p.y()) * px_per_m};
}

Eigen::Vector2d Camera::toWorld(sf::Vector2f p) const {
    return Eigen::Vector2d((p.x - origin_px.x) / px_per_m, (origin_px.y - p.y) / px_per_m);
}

std::optional<sf::Font> loadFont() {
    static const std::array<const char*, 5> candidates{
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial Unicode.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    };
    for (const char* path : candidates) {
        sf::Font font;
        if (font.openFromFile(path)) return font;
    }
    return std::nullopt;
}

void drawWorld(sf::RenderTarget& t, const Camera& cam, const arm::World& world,
               const Palette& pal) {
    for (const arm::Circle& c : world.circles) {
        drawDisc(t, cam.toScreen(c.center), cam.toPixels(c.radius), pal.obstacle, pal.obstacle_edge,
                 1.5f);
    }
    for (const arm::Aabb& b : world.boxes) {
        const sf::Vector2f tl = cam.toScreen(Eigen::Vector2d(b.min.x(), b.max.y()));
        const sf::Vector2f br = cam.toScreen(Eigen::Vector2d(b.max.x(), b.min.y()));
        sf::RectangleShape r({br.x - tl.x, br.y - tl.y});
        r.setPosition(tl);
        r.setFillColor(pal.obstacle);
        r.setOutlineColor(pal.obstacle_edge);
        r.setOutlineThickness(1.5f);
        t.draw(r);
    }
}

void drawWorkspaceBounds(sf::RenderTarget& t, const Camera& cam, const arm::ArmModel& model,
                         const Palette& pal) {
    const sf::Vector2f base = cam.toScreen(model.base.translation());
    // The reachable annulus: outside the outer ring or inside the inner hole,
    // no configuration exists at all.
    drawRing(t, base, cam.toPixels(model.reach()), pal.grid, 1.5f);
    if (model.innerReach() > 1e-6) {
        drawRing(t, base, cam.toPixels(model.innerReach()), pal.grid, 1.5f);
    }
}

void drawArm(sf::RenderTarget& t, const Camera& cam, const arm::ArmModel& model,
             const Eigen::VectorXd& q, sf::Color color, float alpha) {
    const std::vector<Eigen::Vector2d> pts = arm::forwardKinematics(model, q).points();
    const float thickness = std::max(4.f, cam.toPixels(model.link_radius) * 2.f);

    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        drawThickLine(t, cam.toScreen(pts[i]), cam.toScreen(pts[i + 1]), thickness,
                      withAlpha(color, alpha));
    }
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const bool tip = i + 1 == pts.size();
        drawDisc(t, cam.toScreen(pts[i]), tip ? thickness * 0.7f : thickness * 0.55f,
                 withAlpha(tip ? sf::Color(245, 250, 255) : sf::Color(225, 232, 245), alpha));
    }
}

void drawTarget(sf::RenderTarget& t, const Camera& cam, const Eigen::Vector2d& p, sf::Color color) {
    const sf::Vector2f s = cam.toScreen(p);
    drawRing(t, s, 9.f, color, 2.f);
    drawThickLine(t, {s.x - 14.f, s.y}, {s.x + 14.f, s.y}, 1.5f, color);
    drawThickLine(t, {s.x, s.y - 14.f}, {s.x, s.y + 14.f}, 1.5f, color);
}

void drawTrail(sf::RenderTarget& t, const Camera& cam, const std::vector<Eigen::Vector2d>& pts,
               sf::Color color) {
    if (pts.size() < 2) return;
    sf::VertexArray va(sf::PrimitiveType::LineStrip, pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        va[i].position = cam.toScreen(pts[i]);
        va[i].color = withAlpha(color, 0.75f);
    }
    t.draw(va);
}

void drawEllipsoid(sf::RenderTarget& t, const Camera& cam, const Eigen::Vector2d& at,
                   const Eigen::MatrixXd& axes, sf::Color color, double scale) {
    if (axes.rows() < 2 || axes.cols() < 2) return;
    constexpr int kSegments = 96;
    sf::VertexArray va(sf::PrimitiveType::LineStrip, kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
        const double a = 2.0 * M_PI * i / kSegments;
        const Eigen::Vector2d unit(std::cos(a), std::sin(a));
        const Eigen::Vector2d p = at + scale * (axes.topLeftCorner(2, 2) * unit);
        va[i].position = cam.toScreen(p);
        va[i].color = color;
    }
    t.draw(va);

    // The principal axes, so the direction of the lost degree of freedom is
    // visible even when the ellipse collapses to a line.
    for (int c = 0; c < 2; ++c) {
        const Eigen::Vector2d axis = scale * axes.col(c).head(2);
        drawThickLine(t, cam.toScreen(at - axis), cam.toScreen(at + axis), 1.f,
                      withAlpha(color, 0.5f));
    }
}

void drawConditionPlot(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene,
                       const Palette& pal, const sf::Font* font) {
    drawPanel(t, box, pal.panel, pal.grid);
    drawText(t, font, "condition number  sigma_max / sigma_min",
             {box.position.x + 8.f, box.position.y + 6.f}, 12, pal.text_dim);

    const float plot_top = box.position.y + 26.f;
    const float plot_h = box.size.y - 34.f;
    const float plot_w = box.size.x - 16.f;
    const float x0 = box.position.x + 8.f;

    // Log scale from 1 to 1e4: the interesting range is the last decade before
    // the Jacobian goes rank deficient.
    const auto toY = [&](double cond) {
        const double c = std::clamp(std::isfinite(cond) ? cond : 1e4, 1.0, 1e4);
        const double f = std::log10(c) / 4.0;
        return plot_top + plot_h * static_cast<float>(1.0 - f);
    };

    for (int decade = 0; decade <= 4; ++decade) {
        const float y = toY(std::pow(10.0, decade));
        drawThickLine(t, {x0, y}, {x0 + plot_w, y}, 1.f, pal.grid);
        drawText(t, font, "1e" + std::to_string(decade), {x0 + plot_w - 24.f, y - 12.f}, 10,
                 pal.text_dim);
    }

    if (scene.cond_history.size() >= 2) {
        sf::VertexArray va(sf::PrimitiveType::LineStrip, scene.cond_history.size());
        const std::size_t n = scene.cond_history.size();
        for (std::size_t i = 0; i < n; ++i) {
            // Newest sample at the right edge, so a short history still reads.
            const float age = static_cast<float>(n - 1 - i) / (scene.cond_history_max - 1);
            va[i].position = {x0 + (1.f - age) * plot_w, toY(scene.cond_history[i])};
            va[i].color = scene.cond_history[i] > 50.0 ? pal.accent : pal.link;
        }
        t.draw(va);
    }
}

void drawJointSpace(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene,
                    const Palette& pal, const sf::Font* font) {
    drawPanel(t, box, pal.panel, pal.grid);

    if (scene.model.dof() != 2 || scene.cspace_res == 0) {
        drawText(t, font, "joint space view needs a 2R arm  (press 2)",
                 {box.position.x + 16.f, box.position.y + 16.f}, 14, pal.text_dim);
        return;
    }

    const int res = scene.cspace_res;
    sf::Image img(sf::Vector2u{static_cast<unsigned>(res), static_cast<unsigned>(res)},
                  pal.panel);
    for (int i = 0; i < res; ++i) {      // theta1 -> x
        for (int j = 0; j < res; ++j) {  // theta2 -> y, drawn upward
            const unsigned char flag = scene.cspace[static_cast<std::size_t>(i) * res + j];
            sf::Color c = pal.panel;
            if (flag == 1) c = pal.obstacle;        // C-space obstacle
            else if (flag == 2) c = sf::Color(40, 40, 48);  // outside joint limits
            img.setPixel({static_cast<unsigned>(i), static_cast<unsigned>(res - 1 - j)}, c);
        }
    }
    sf::Texture tex(img);
    tex.setSmooth(true);
    sf::Sprite sprite(tex);
    const float side = std::min(box.size.x, box.size.y) - 96.f;
    sprite.setScale({side / res, side / res});
    const sf::Vector2f plot_origin{box.position.x + 44.f, box.position.y + 24.f};
    sprite.setPosition(plot_origin);
    t.draw(sprite);

    const auto toPix = [&](const Eigen::VectorXd& c) {
        const double u = (arm::wrapAngle(c[0]) + M_PI) / (2.0 * M_PI);
        const double v = (arm::wrapAngle(c[1]) + M_PI) / (2.0 * M_PI);
        return sf::Vector2f{plot_origin.x + static_cast<float>(u) * side,
                            plot_origin.y + static_cast<float>(1.0 - v) * side};
    };

    // Both interpolation ghosts leave a track through the same C-space.
    if (scene.demo.ready()) {
        drawDisc(t, toPix(scene.demo.qa), 5.f, pal.text);
        drawDisc(t, toPix(scene.demo.qb), 5.f, pal.target);
        if (scene.demo.q_joint.size() == 2) drawDisc(t, toPix(scene.demo.q_joint), 4.f, pal.ghost_joint);
        if (scene.demo.q_cart.size() == 2) drawDisc(t, toPix(scene.demo.q_cart), 4.f, pal.ghost_cart);
    }
    drawDisc(t, toPix(scene.q), 6.f, scene.collision.hit() ? pal.link_bad : pal.link);

    drawText(t, font, "theta1", {plot_origin.x + side * 0.5f - 16.f, plot_origin.y + side + 6.f}, 12,
             pal.text_dim);
    drawText(t, font, "theta2", {box.position.x + 6.f, plot_origin.y + side * 0.5f}, 12,
             pal.text_dim);
    drawText(t, font, "-pi", {plot_origin.x - 6.f, plot_origin.y + side + 6.f}, 10, pal.text_dim);
    drawText(t, font, "+pi", {plot_origin.x + side - 12.f, plot_origin.y + side + 6.f}, 10,
             pal.text_dim);
    drawText(t, font, "shaded: configurations in collision   dark border: outside joint limits",
             {plot_origin.x, plot_origin.y + side + 24.f}, 11, pal.text_dim);
}

void drawHud(sf::RenderTarget& t, const sf::FloatRect& box, const Scene& scene, const Palette& pal,
             const sf::Font* font) {
    drawPanel(t, box, pal.panel, pal.grid);
    const arm::Manipulability m = scene.manipulability();

    float y = box.position.y + 10.f;
    const float x = box.position.x + 12.f;

    y = drawText(t, font, "arm-kinematics", {x, y}, 16, pal.text);
    y = drawText(t, font, std::to_string(scene.model.dof()) + "R planar   reach " +
                              fmt(scene.model.reach(), 2) + " m",
                 {x, y}, 12, pal.text_dim);
    y += 6.f;

    y = drawText(t, font, "sigma_min   " + fmt(m.sigma_min), {x, y}, 13,
                 m.sigma_min < 0.05 ? pal.accent : pal.text);
    y = drawText(t, font, "sigma_max   " + fmt(m.sigma_max), {x, y}, 13, pal.text);
    y = drawText(t, font, "cond        " + fmt(m.condition_number, 1), {x, y}, 13,
                 m.condition_number > 50.0 ? pal.accent : pal.text);
    y = drawText(t, font, "yoshikawa   " + fmt(m.yoshikawa), {x, y}, 13, pal.text);
    y += 6.f;

    y = drawText(t, font,
                 std::string("target      ") + (scene.target_reachable ? "reachable" : "OUT OF REACH"),
                 {x, y}, 13, scene.target_reachable ? pal.text : pal.accent);
    y = drawText(t, font, "ik error    " + fmt(scene.ik_error, 5), {x, y}, 13,
                 scene.ik_converged ? pal.text : pal.accent);

    std::string collision_text = "clear";
    sf::Color collision_color = pal.text;
    if (scene.collision.kind == arm::CollisionReport::Kind::Obstacle) {
        collision_text = "link " + std::to_string(scene.collision.link) + " vs obstacle " +
                         std::to_string(scene.collision.obstacle);
        collision_color = pal.link_bad;
    } else if (scene.collision.kind == arm::CollisionReport::Kind::Self) {
        collision_text = "self: links " + std::to_string(scene.collision.link) + " and " +
                         std::to_string(scene.collision.other_link);
        collision_color = pal.link_bad;
    }
    y = drawText(t, font, "collision   " + collision_text, {x, y}, 13, collision_color);
    y += 10.f;

    if (scene.playback.active) {
        const SupervisorPlayback& p = scene.playback;
        y = drawText(t, font, "supervisor playback", {x, y}, 14, pal.text);
        y = drawText(t, font, "  supervised", {x, y}, 12, pal.link);
        y = drawText(t, font, "  unsupervised, same commands", {x, y}, 12, pal.link_bad);
        y += 4.f;
        y = drawText(t, font, "commands    " + std::to_string(p.commands), {x, y}, 13, pal.text);
        y = drawText(t, font, "intervened  " + std::to_string(p.interventions), {x, y}, 13,
                     pal.text);
        y = drawText(t, font, "rejected    " + std::to_string(p.rejections), {x, y}, 13, pal.text);
        y = drawText(t, font, "violations, unsupervised   " + std::to_string(p.raw_violations),
                     {x, y}, 13, pal.link_bad);
        y = drawText(t, font, "last check  " + std::string(checkName(p.last.failed_check)), {x, y},
                     13, p.last.failed_check == arm::Check::None ? pal.text : pal.accent);
        y = drawText(t, font, "fallback    " +
                                  std::string(p.last.fallback_engaged
                                                  ? fallbackName(p.last.fallback)
                                                  : "none"),
                     {x, y}, 13, pal.text);
        y = drawText(t, font, "latency     " + fmt(p.last.latency_us, 2) + " us", {x, y}, 13,
                     pal.text);
        y += 4.f;
        for (const std::string& line : p.events) {
            y = drawText(t, font, "  " + line, {x, y}, 11, pal.text_dim);
        }
    } else if (scene.demo.ready()) {
        y = drawText(t, font, "dual interpolation  t = " + fmt(scene.demo.t, 2), {x, y}, 13,
                     pal.text);
        y = drawText(t, font, "  joint space", {x, y}, 12, pal.ghost_joint);
        y = drawText(t, font,
                     std::string("  cartesian") +
                         (scene.demo.cart_ok ? "" : "  LOST TRACKING"),
                     {x, y}, 12, scene.demo.cart_ok ? pal.ghost_cart : pal.accent);
    } else {
        y = drawText(t, font, "dual interpolation: set A and B", {x, y}, 12, pal.text_dim);
    }
    y += 10.f;

    for (const char* line : {"drag        move target", "A / B       capture pose",
                             "space       play interpolation", "S           supervisor playback",
                             "J           joint space view", "2 / 3       switch arm",
                             "E           ellipsoid", "right-click add obstacle",
                             "R           reset"}) {
        y = drawText(t, font, line, {x, y}, 11, pal.text_dim);
    }
}

void drawFrame(sf::RenderTarget& t, const Scene& scene, const Camera& cam, const Palette& pal,
               const sf::Font* font) {
    t.clear(pal.background);

    const sf::Vector2f size = sf::Vector2f(t.getSize());
    const float panel_w = 320.f;
    const sf::FloatRect main{{0.f, 0.f}, {size.x - panel_w, size.y}};

    if (scene.view == View::JointSpace) {
        drawJointSpace(t, main, scene, pal, font);
    } else {
        drawWorkspaceBounds(t, cam, scene.model, pal);
        drawWorld(t, cam, scene.world, pal);

        if (scene.playback.active && scene.playback.q_raw.size() == scene.q.size()) {
            // The same command stream driving both arms: the unsupervised one
            // is what the supervisor is being compared against, not a ghost.
            drawArm(t, cam, scene.model, scene.playback.q_raw, pal.link_bad, 0.55f);
        }

        if (scene.demo.ready() && scene.demo.q_joint.size() == scene.q.size()) {
            drawTrail(t, cam, scene.demo.trail_joint, pal.ghost_joint);
            drawTrail(t, cam, scene.demo.trail_cart, pal.ghost_cart);
            drawArm(t, cam, scene.model, scene.demo.q_joint, pal.ghost_joint, 0.75f);
            drawArm(t, cam, scene.model, scene.demo.q_cart, pal.ghost_cart, 0.75f);
        }

        drawArm(t, cam, scene.model, scene.q, scene.collision.hit() ? pal.link_bad : pal.link);
        const bool refused =
            scene.playback.active && scene.playback.last.failed_check != arm::Check::None;
        drawTarget(t, cam, scene.target, refused ? pal.accent : pal.target);
        if (refused && font) {
            const std::string label = std::string("rejected: ") +
                                      checkName(scene.playback.last.failed_check) + "  ->  " +
                                      fallbackName(scene.playback.last.fallback);
            const sf::Vector2f at = cam.toScreen(scene.target);
            // Flip the label to the other side of the target rather than let it
            // run under the panel.
            const float width_estimate = label.size() * 6.5f;
            const float px = at.x + 16.f + width_estimate < main.size.x
                                 ? at.x + 16.f
                                 : at.x - 16.f - width_estimate;
            drawText(t, font, label, {std::max(4.f, px), at.y - 8.f}, 13, pal.accent);
        }

        if (scene.show_ellipsoid) {
            const arm::Manipulability m = scene.manipulability();
            drawEllipsoid(t, cam, arm::eePosition(scene.model, scene.q), m.ellipsoid_axes,
                          pal.ellipsoid);
        }
    }

    const sf::FloatRect hud{{size.x - panel_w + 8.f, 8.f}, {panel_w - 16.f, size.y - 234.f}};
    drawHud(t, hud, scene, pal, font);
    const sf::FloatRect plot{{size.x - panel_w + 8.f, size.y - 218.f}, {panel_w - 16.f, 210.f}};
    drawConditionPlot(t, plot, scene, pal, font);
}

}  // namespace viz
