#pragma once

#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include "include/transform_stack.hpp"

class Frame
{
    public:
        Frame();
        Frame(Eigen::Vector2d pos, float angle);
        virtual ~Frame();

        float angle = 0;
        Eigen::Vector2d pos;

        void draw(sf::RenderWindow& window, TransformStack& ts);

    protected:

    private:
};
