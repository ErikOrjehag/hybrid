#include "../include/transform_stack.hpp"

namespace dyno
{

namespace visual
{

void TransformStack::x_up_y_left_centered(const sf::Vector2u& windowSize, unsigned px_per_meter)
{
    translate(windowSize.x / 2, windowSize.y / 2);
    rotate(-90);
    scale(1, -1);
    scale(px_per_meter);
}

void TransformStack::translate(float x, float y)
{
    transformCur.translate(x,y);
}

void TransformStack::rotate(float angle)
{
    transformCur.rotate(angle);
}

void TransformStack::scale(float sx, float sy)
{
    transformCur.scale(sx, sy);
}

void TransformStack::scale(float s)
{
    scale(s,s);
}

void TransformStack::push()
{
    transformTot = transformTot.combine(transformCur);
    stack.push(transformCur);
    transformCur = sf::Transform();
}

void TransformStack::pop()
{
    sf::Transform transform = stack.top();
    stack.pop();
    sf::Transform invTransform = transform.getInverse();
    transformTot = transformTot.combine(invTransform);
    transformCur = transform;
}

Eigen::Vector2d TransformStack::transform_point(const Eigen::Vector2d& point)
{
    sf::Vector2f v = sf::Transform(*this).getInverse() * sf::Vector2f(point.x(), point.y());
    return Eigen::Vector2d(v.x, v.y);
}

TransformStack::operator sf::Transform()
{
    return transformTot * transformCur;
}

TransformStack::operator sf::RenderStates()
{
    return static_cast<sf::RenderStates>( operator sf::Transform() );
}

} // namespace visual

} // namespace dyno