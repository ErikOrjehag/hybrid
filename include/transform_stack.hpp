#pragma once

#include <SFML/Graphics.hpp>
#include <stack>
#include <Eigen/Dense>

namespace dyno
{

namespace visual
{

class TransformStack
{
    private:
        sf::Transform transformCur;
        sf::Transform transformTot;
        std::stack<sf::Transform> stack;
    public:
        // Initialization
        void x_up_y_left_centered(const sf::Vector2u& windowSize, unsigned px_per_meter = 50);
        
        // Transformations
        void translate(float x, float y);
        void rotate(float angle);
        void scale(float sx, float sy);
        void scale(float s);
        void push();
        void pop();

        // Lookups
        Eigen::Vector2d transform_point(const Eigen::Vector2d& point);

        // Castings
        operator sf::Transform();
        operator sf::RenderStates();
};

} // namespace visual

} // namespace dyno