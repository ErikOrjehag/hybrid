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
    GridMapRenderer(GridMap& grid_map)
    {
        update(grid_map);
    }

    void update(GridMap& grid_map)
    {
        if (!texture_.create(grid_map.width(), grid_map.height()))
        {
            throw std::runtime_error("Failed to create texture");
        }

        std::vector<sf::Uint8> pixels(grid_map.width() * grid_map.height() * 4);

        for (size_t row = 0; row < grid_map.height(); ++row)
        {
            for (size_t col = 0; col < grid_map.width(); ++col)
            {
                sf::Uint8 val = static_cast<sf::Uint8>(std::round((/* 1.0 - */grid_map.at(row, col)) * 255.0));
                size_t pixels_idx = (row * grid_map.width() + col) * 4;
                pixels[pixels_idx + 0] = val;
                pixels[pixels_idx + 1] = val;
                pixels[pixels_idx + 2] = val;
                pixels[pixels_idx + 3] = 255;
            }
        }

        texture_.update(pixels.data());
        texture_.setSmooth(false);
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
};

} // namespace visual
} // namespace dyno