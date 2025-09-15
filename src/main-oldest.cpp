

#if 0

#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/frame.hpp"
#include "include/heap.hpp"
#include "include/node_3d.hpp"
#include <math.h>
#include <iostream>

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;
const int PX_PER_METER = 25;

double heuristic(const Node3D* start, const Node3D* goal)
{
    return std::sqrt(std::pow(start->x - goal->x, 2) + std::pow(start->y - goal->y, 2));
}

void hybrid_a_star(sf::RenderWindow& window, TransformStack& ts, sf::Vector3f s, sf::Vector3f g, std::vector<sf::Vector3f>& path, const size_t MAX)
{
    path.clear();

    size_t width = 30;
    size_t height = 30;
    size_t depth = 20;
    Node3D* nodes3D = new Node3D[width * height * depth]();
    Node3D start(s.x, s.y, s.z, 0.0, 0.0, nullptr);
    Node3D goal(g.x, g.y, g.z, 0.0, 0.0, nullptr);

    PriorityQueue open;

    start.update_index(width, height, depth);
    start.h = heuristic(&start, &goal);
    start.open();

    nodes3D[start.index] = start;
    /*nodes3D[start.index].handle = */open.push(&nodes3D[start.index]);

    goal.update_index(width, height, depth);
    nodes3D[goal.index] = goal;

    // TODO: prepare start and goal but don't have them in scope, only have nodes3D and indexes referring to that data

    size_t iter = 0;

    while (!open.empty() && iter < MAX)
    {
        iter += 1;

        Node3D* top = open.top();
        open.pop();

        if (top->is_closed()) {
            {
                sf::Color c(sf::Color::Magenta);

                sf::Vertex xAxis[] =
                {
                    sf::Vertex(sf::Vector2f(-1.414, 0), c),
                    sf::Vertex(sf::Vector2f(0, 0), c),
                    sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                    sf::Vertex(sf::Vector2f(0, 0), c),
                    sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                    sf::Vertex(sf::Vector2f(0, 0), c)
                };

                ts.push();
                ts.translate(top->x, top->y);
                ts.rotate(top->t * 180 / M_PI);
                window.draw(xAxis, 6, sf::Lines, ts);
                ts.pop();
            }
            continue;
        }

        top->close();

        if (top->index == goal.index) // || iterations > MAX
        {
            // backtrace path
            Node3D* current = top;
            while (current != nullptr)
            {
                path.push_back(sf::Vector3f(current->x, current->y, current->t));
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        // dubins shot

        for (size_t i = 0; i < Node3D::N_DIRS; i++)
        {
            Node3D* child = top->create_child(i);
            size_t child_index = child->update_index(width, height, depth);

            if (child->x < 0.0 || child->x >= width || child->y < 0.0 || child->y >= height)
            {
                {
                    sf::Color c(sf::Color::Red);

                    sf::Vertex xAxis[] =
                    {
                        sf::Vertex(sf::Vector2f(-1.414, 0), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c)
                    };

                    ts.push();
                    ts.translate(child->x, child->y);
                    ts.rotate(child->t * 180 / M_PI);
                    window.draw(xAxis, 6, sf::Lines, ts);
                    ts.pop();
                }
                delete child;
                continue;
            }

            if (nodes3D[child_index].is_closed() && top->index != child_index)
            {
                {
                   sf::Color c(sf::Color::Cyan);

                    sf::Vertex xAxis[] =
                    {
                        sf::Vertex(sf::Vector2f(-1.414, 0), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c)
                    };

                    ts.push();
                    ts.translate(child->x, child->y);
                    ts.rotate(child->t * 180 / M_PI);
                    window.draw(xAxis, 6, sf::Lines, ts);
                    ts.pop();
                }
                delete child;
                continue;
            }

            child->update_g();

            if (nodes3D[child_index].is_open() && child->g >= nodes3D[child_index].g && top->index != child_index)
            {
                {
                   sf::Color c(sf::Color::Green);

                    sf::Vertex xAxis[] =
                    {
                        sf::Vertex(sf::Vector2f(-1.414, 0), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c),
                        sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                        sf::Vertex(sf::Vector2f(0, 0), c)
                    };

                    ts.push();
                    ts.translate(child->x, child->y);
                    ts.rotate(child->t * 180 / M_PI);
                    window.draw(xAxis, 6, sf::Lines, ts);
                    ts.pop();
                }
                delete child;
                continue;
            }

            child->h = heuristic(child, &goal);

            if (top->index == child->index)
            {
                if (child->cost() > top->cost() + 0.01) { // Constants::tieBreaker
                    {
                        sf::Color c(sf::Color::Blue);

                        sf::Vertex xAxis[] =
                        {
                            sf::Vertex(sf::Vector2f(-1.414, 0), c),
                            sf::Vertex(sf::Vector2f(0, 0), c),
                            sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                            sf::Vertex(sf::Vector2f(0, 0), c),
                            sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                            sf::Vertex(sf::Vector2f(0, 0), c)
                        };

                        ts.push();
                        ts.translate(child->x, child->y);
                        ts.rotate(child->t * 180 / M_PI);
                        window.draw(xAxis, 6, sf::Lines, ts);
                        ts.pop();
                    }
                    delete child;
                    continue;
                } else {
                    child->parent = top->parent;
                }
            }

            {
                sf::Color c(sf::Color::Yellow);

                sf::Vertex xAxis[] =
                {
                    sf::Vertex(sf::Vector2f(-1.414, 0), c),
                    sf::Vertex(sf::Vector2f(0, 0), c),
                    sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
                    sf::Vertex(sf::Vector2f(0, 0), c),
                    sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
                    sf::Vertex(sf::Vector2f(0, 0), c)
                };

                ts.push();
                ts.translate(child->x, child->y);
                ts.rotate(child->t * 180 / M_PI);
                window.draw(xAxis, 6, sf::Lines, ts);
                ts.pop();
            }

            child->open();
            nodes3D[child_index] = *child;
            /* child.handle = */ open.push(&nodes3D[child_index]);
            delete child;
        }

        // {
        //     sf::Color c(sf::Color::White);

        //     sf::Vertex xAxis[] =
        //     {
        //         sf::Vertex(sf::Vector2f(-1.414, 0), c),
        //         sf::Vertex(sf::Vector2f(0, 0), c),
        //         sf::Vertex(sf::Vector2f(-0.2, 0.1), c),
        //         sf::Vertex(sf::Vector2f(0, 0), c),
        //         sf::Vertex(sf::Vector2f(-0.2, -0.1), c),
        //         sf::Vertex(sf::Vector2f(0, 0), c)
        //     };

        //     ts.push();
        //     ts.translate(top->x, top->y);
        //     ts.rotate(top->t * 180 / M_PI);
        //     window.draw(xAxis, 6, sf::Lines, ts);
        //     ts.pop();
        // }
    }

    delete[] nodes3D;
}

enum ControlState {
    SET_START,
    SET_GOAL,
};

int main()
{
    ControlState controlState = SET_START;
    sf::Vector2f mousePosPress(0, 0);
    sf::Vector3f start(10, 10, 0);
    sf::Vector3f goal(20, 12, 0);
    std::vector<sf::Vector3f> path;
    int max_iter = 0;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "MPC", sf::Style::Default, settings);

    sf::Clock clock;

    TransformStack ts;
    ts.rotate(-90);
    ts.scale(1, -1);
    ts.translate(5*WINDOW_HEIGHT/-6, 5*WINDOW_WIDTH/-6);
    ts.scale(PX_PER_METER, PX_PER_METER);

    Frame globalFrame;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Num1)
                {
                    controlState = SET_START;
                }
                else if (event.key.code == sf::Keyboard::Num2)
                {
                    controlState = SET_GOAL;
                }
                else if (event.key.code == sf::Keyboard::Up)
                {
                    max_iter += 1;
                }
                else if (event.key.code == sf::Keyboard::Down)
                {
                    max_iter = std::max(0, max_iter - 1);
                }
            }
            else if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Button::Left)
                {
                    mousePosPress = sf::Transform(ts).getInverse() * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
                }
            }
            else if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Button::Left)
                {
                    sf::Vector2f mousePosRelease = sf::Transform(ts).getInverse() * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
                    sf::Vector2f mousePosDelta = mousePosRelease - mousePosPress;
                    float theta = atan2(mousePosDelta.y, mousePosDelta.x);
                    if (controlState == SET_GOAL)
                    {
                        goal.x = mousePosPress.x;
                        goal.y = mousePosPress.y;
                        goal.z = theta;
                    }
                    else if (controlState == SET_START)
                    {
                        start.x = mousePosPress.x;
                        start.y = mousePosPress.y;
                        start.z = theta;
                    }
                }
            }
        }

        float dt = clock.restart().asSeconds();

        window.clear(sf::Color(50, 50, 50, 255));

        globalFrame.draw(window, ts);
        
        hybrid_a_star(window, ts, start, goal, path, max_iter);

        if (path.size() > 0)
        {
            sf::Vertex line[path.size()];
            for (size_t i = 0; i < path.size(); i++)
            {
                line[i] = sf::Vertex(sf::Vector2f(path[i].x, path[i].y), sf::Color::Red);
            }
            window.draw(line, path.size(), sf::LinesStrip, ts);
        }

        {
            sf::Vertex startAxis[] =
            {
                sf::Vertex(sf::Vector2f(-0.2, 0), sf::Color::Blue),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
                sf::Vertex(sf::Vector2f(-0.1, 0.1), sf::Color::Blue),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
                sf::Vertex(sf::Vector2f(-0.1, -0.1), sf::Color::Blue),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue)
            };

            sf::Vertex goalAxis[] =
            {
                sf::Vertex(sf::Vector2f(-0.2, 0), sf::Color::Green),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
                sf::Vertex(sf::Vector2f(-0.1, 0.1), sf::Color::Green),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
                sf::Vertex(sf::Vector2f(-0.1, -0.1), sf::Color::Green),
                sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green)
            };

            ts.push();
            ts.translate(start.x, start.y);
            ts.rotate(start.z * 180 / M_PI);
            window.draw(startAxis, 6, sf::Lines, ts);
            ts.pop();
            ts.push();
            ts.translate(goal.x, goal.y);
            ts.rotate(goal.z * 180 / M_PI);
            window.draw(goalAxis, 6, sf::Lines, ts);
            ts.pop();
        }
        
        window.display();
    }

    return 0;
}
#endif