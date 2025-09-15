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
        void translate(double x, double y);
        void rotate(double angle);
        void scale(double sx, double sy);
        void scale(double s);
        void push();
        void pop();

        // Lookups
        Eigen::Vector2d transform_point(const Eigen::Vector2d& point);

        // Castings
        operator sf::Transform();
        operator sf::RenderStates();
};

struct TransformArgs {
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;
    double s = 0.0;
    double sx = 1.0;
    double sy = 1.0;
};

void transform(TransformStack& ts, TransformArgs args, std::function<void()> func);

} // namespace visual

} // namespace dyno