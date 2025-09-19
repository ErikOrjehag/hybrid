#include "../include/transform_stack.hpp"

namespace dyno
{

namespace visual
{

void TransformStack::x_up_y_left_centered(const sf::Vector2u& windowSize, unsigned px_per_meter)
{
    translate(windowSize.x / 2, windowSize.y / 2);
    rotate(-M_PI_2);
    scale(1, -1);
    scale(px_per_meter);
}

void TransformStack::translate(double x, double y)
{
    transformCur.translate(x,y);
}

void TransformStack::rotate(double angle)
{
    // angle in radians
    transformCur.rotate(angle * 180.0 / M_PI);
}

void TransformStack::scale(double sx, double sy)
{
    transformCur.scale(sx, sy);
}

void TransformStack::scale(double s)
{
    scale(s, s);
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
    
    if (stack.empty())
    {
        transformTot = sf::Transform();
    }
    else
    {
        sf::Transform invTransform = transform.getInverse();
        transformTot = transformTot.combine(invTransform);
    }

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

void transform(TransformStack& ts, TransformArgs args, std::function<void()> func)
{
    bool need_push = args.x || args.y || args.angle || args.s || args.sx != 1.0 || args.sy != 1.0;
    if (!need_push)
    {
        func();
        return;
    }

    ts.push();
    if (args.x || args.y)
    {
        ts.translate(args.x, args.y);
    }
    if (args.angle)
    {
        ts.rotate(args.angle);
    }
    if (args.s) {
        ts.scale(args.s);
    } else if (args.sx != 1.0 || args.sy != 1.0) {
        ts.scale(args.sx, args.sy);
    }
    func();
    ts.pop();
}

} // namespace visual

} // namespace dyno