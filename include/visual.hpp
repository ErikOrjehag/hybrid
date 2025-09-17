#pragma once

#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include <cmath>
#include "include/grid_map.hpp"

namespace dyno
{
namespace visual
{
const sf::Vertex X_AXIS[] =
{
    sf::Vertex(sf::Vector2f(0, 0), sf::Color::Red),
    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red),
    sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Red),
    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red),
    sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Red),
    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Red)
};

const sf::Vertex Y_AXIS[] =
{
    sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
    sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green),
    sf::Vertex(sf::Vector2f(0.1, 0.9), sf::Color::Green),
    sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green),
    sf::Vertex(sf::Vector2f(-0.1, 0.9), sf::Color::Green),
    sf::Vertex(sf::Vector2f(0, 1), sf::Color::Green)
};

const sf::Vertex GRID_LINE[] = 
{
    sf::Vertex(sf::Vector2f(0, -1), sf::Color(80, 80, 80, 255)),
    sf::Vertex(sf::Vector2f(0, 1), sf::Color(80, 80, 80, 255))
};

void draw_frame(sf::RenderWindow& window, dyno::visual::TransformStack& ts, double x = 0.0, double y = 0.0, double angle = 0.0)
{
    transform(ts, {.x=x, .y=y, .angle=angle}, [&]() {
        window.draw(X_AXIS, 6, sf::Lines, ts);
        window.draw(Y_AXIS, 6, sf::Lines, ts);
    });
}

void draw_point(sf::RenderWindow& window, dyno::visual::TransformStack& ts, sf::Color color = sf::Color::White, double r = 1.0, double x = 0.0, double y = 0.0)
{
    transform(ts, {.x=x-r/2.0, .y=y-r/2.0}, [&]() {
        sf::CircleShape point(r, 9);
        point.setFillColor(color);
        window.draw(point, ts);
    });
}

void draw_grid(sf::RenderWindow& window, dyno::visual::TransformStack& ts, size_t size = 10)
{
    transform(ts, {.x = -static_cast<float>(size), .sy=static_cast<double>(size)}, [&]() {
        for (size_t _ = 0; _ <= 2*size; _ += 1)
        {
            window.draw(GRID_LINE, 2, sf::Lines, ts);
            ts.translate(1, 0);
        }
    });


    ts.push();
    ts.translate(0, -static_cast<float>(size));
    ts.rotate(M_PI_2);
    ts.scale(1, size);
    for (size_t _ = 0; _ <= 2*size; _ += 1)
    {
        window.draw(GRID_LINE, 2, sf::Lines, ts);
        ts.translate(1, 0);
    }
    ts.pop();
}

class GridMapRenderer
{
public:
    GridMapRenderer(GridMap& grid_map, sf::Color low_color, sf::Color high_color, double alpha = 1.0) : low_color_(low_color), high_color_(high_color)
    {
        texture_.setSmooth(false);
        sprite_.setColor(sf::Color(255, 255, 255, alpha * 255));
        update(grid_map);
    }

    void update(GridMap& grid_map)
    {
        if (!texture_.create(grid_map.width(), grid_map.height()))
        {
            throw std::runtime_error("Failed to create texture");
        }

        auto rgbToLinear = [](sf::Uint8 c) {
            double c_srgb = static_cast<double>(c) / 255.0;
            if (c_srgb <= 0.04045)
                return c_srgb / 12.92;
            else
                return std::pow((c_srgb + 0.055) / 1.055, 2.4);
        };
        auto linearToRgb = [](double c) {
            if (c <= 0.0031308)
                c = c * 12.92;
            else
                c = 1.055 * std::pow(c, 1.0 / 2.4) - 0.055;
            return static_cast<sf::Uint8>(std::round(std::min(std::max(c, 0.0), 1.0) * 255.0));
        };
        auto interpolateColor = [&](sf::Color c1, sf::Color c2, double t) {
            double r = rgbToLinear(c1.r) * (1.0 - t) + rgbToLinear(c2.r) * t;
            double g = rgbToLinear(c1.g) * (1.0 - t) + rgbToLinear(c2.g) * t;
            double b = rgbToLinear(c1.b) * (1.0 - t) + rgbToLinear(c2.b) * t;
            double a = (c1.a / 255.0) * (1.0 - t) + (c2.a / 255.0) * t;
            sf::Uint8 alpha = static_cast<sf::Uint8>(std::round(std::min(std::max(a, 0.0), 1.0) * 255.0));
            return sf::Color(linearToRgb(r), linearToRgb(g), linearToRgb(b), alpha);
        };

        std::vector<sf::Uint8> pixels(grid_map.width() * grid_map.height() * 4);

        double min_val = grid_map.min();
        double max_val = grid_map.max();

        for (size_t row = 0; row < grid_map.height(); ++row)
        {
            for (size_t col = 0; col < grid_map.width(); ++col)
            {
                double v = grid_map.at(row, col);
                v = (v - min_val);
                double devisor = (max_val - min_val);
                if (devisor > 1e-6)
                    v /= devisor;
                else
                    v = 0.0;
                v = std::min(std::max(v, 0.0), 1.0);
                sf::Color mixed_color = interpolateColor(low_color_, high_color_, v);
                size_t pixels_idx = (row * grid_map.width() + col) * 4;
                pixels[pixels_idx + 0] = mixed_color.r;
                pixels[pixels_idx + 1] = mixed_color.g;
                pixels[pixels_idx + 2] = mixed_color.b;
                pixels[pixels_idx + 3] = mixed_color.a;
            }
        }

        texture_.update(pixels.data());
        sprite_.setTexture(texture_);
        grid_map_ = grid_map;
    }

    void draw(sf::RenderWindow& window, dyno::visual::TransformStack& ts)
    {
        transform(ts, {.x=grid_map_.x(), .y=grid_map_.y(), .s=grid_map_.resolution()}, [&]() {
            window.draw(sprite_, ts);
        });
    }
private:
    dyno::GridMap grid_map_;
    sf::Texture texture_;
    sf::Sprite sprite_;
    sf::Color low_color_;
    sf::Color high_color_;
};

} // namespace visual
} // namespace dyno