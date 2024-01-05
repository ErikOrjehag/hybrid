#pragma once

#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include <cmath>

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

        void draw_frame(sf::RenderWindow& window, dyno::visual::TransformStack& ts, double x = 0.0, double y = 0.0, double angle = 0.0)
        {
            ts.push();
            ts.translate(x, y);
            ts.rotate(angle * 180 / M_PI);
            window.draw(X_AXIS, 6, sf::Lines, ts);
            window.draw(Y_AXIS, 6, sf::Lines, ts);
            ts.pop();
        }
    }
}