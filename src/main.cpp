#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/frame.hpp"
#include "include/heap.hpp"
#include "include/node_3d.hpp"
#include <math.h>
#include <iostream>

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;
const int PX_PER_METER = 10;

double heuristic(const Node3D* start, const Node3D* goal)
{
    return std::sqrt(std::pow(start->x - goal->x, 2) + std::pow(start->y - goal->y, 2));
}

Node3D* hybrid_a_star(sf::RenderWindow& window, TransformStack& ts)
{
    size_t width = 30;
    size_t height = 30;
    size_t depth = 20;
    Node3D* nodes3D = new Node3D[width * height * depth]();
    Node3D start(10.0, 10.0, 0.0, 0.0, 0.0, nullptr);
    Node3D goal(20.0, 12.0, 0.0, 0.0, 0.0, nullptr);

    PriorityQueue open;

    start.update_index(width, height, depth);
    start.h = heuristic(&start, &goal);
    start.open();

    nodes3D[start.index] = start;
    /*nodes3D[start.index].handle = */open.push(&nodes3D[start.index]);

    goal.update_index(width, height, depth);
    nodes3D[goal.index] = goal;

    // TODO: prepare start and goal but don't have them in scope, only have nodes3D and indexes refering to that data

    while (!open.empty())
    {
        Node3D* top = open.top();
        open.pop();

        if (top->is_closed()) {
            continue;
        }

        top->close();

        {
            sf::Color c(sf::Color::Red);
            //sf::Color c(255, 0, 0, std::max<int>(0, std::min<int>(255, (int)(255*((20.0 - top->cost())/20.0)))));
            // std::cout << "cost: " << top->cost() << ", g: " << top->g << ", h: " << top->h << std::endl;

            sf::Vertex xAxis[] =
            {
                sf::Vertex(sf::Vector2f(0, 0), c),
                sf::Vertex(sf::Vector2f(1, 0), c),
                sf::Vertex(sf::Vector2f(0.9, 0.1), c),
                sf::Vertex(sf::Vector2f(1, 0), c),
                sf::Vertex(sf::Vector2f(0.9, -0.1), c),
                sf::Vertex(sf::Vector2f(1, 0), c)
            };

            ts.push();
            ts.translate(top->x, top->y);
            ts.rotate(top->t * 180 / M_PI);
            window.draw(xAxis, 6, sf::Lines, ts);
            ts.pop();
        }

        if (top->index == goal.index) // || iterations > MAX
        {
            std::cout << "goal!" << std::endl;
            {
                sf::Vertex startAxis[] =
                {
                    sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue)
                };

                sf::Vertex goalAxis[] =
                {
                    sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Green),
                    sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green)
                };

                ts.push();
                ts.translate(start.x, start.y);
                ts.rotate(start.t * 180 / M_PI);
                window.draw(startAxis, 6, sf::Lines, ts);
                ts.pop();
                ts.push();
                ts.translate(goal.x, goal.y);
                ts.rotate(goal.t * 180 / M_PI);
                window.draw(goalAxis, 6, sf::Lines, ts);
                ts.pop();
            }
            // packtrace path
            delete[] nodes3D;
            return top;
        }

        // dubins shot

        for (size_t i = 0; i < Node3D::N_DIRS; i++)
        {
            Node3D* child = top->create_child(i);
            size_t child_index = child->update_index(width, height, depth);

            if (child->x < 0.0 || child->x >= width || child->y < 0.0 || child->y >= height) // !(child->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc))
            {
                delete child;
                continue;
            }

            // std::cout << child_index << std::endl;
            //std::cout << !nodes3D[child_index].is_closed() << std::endl;

            // !nodes3D[iSucc].isClosed() || iPred == iSucc
            // !(!nodes3D[iSucc].isClosed() || iPred == iSucc)
            // nodes3D[iSucc].isClosed() && iPred != iSucc)

            if (nodes3D[child_index].is_closed() && top->index != child_index)
            {
                delete child;
                continue;
            }

            child->update_g();

            // !nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc
            // !(!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc)
            // nodes3D[iSucc].isOpen() && newG >= nodes3D[iSucc].getG() && iPred != iSucc
            if (child->is_open() && child->g >= nodes3D[child_index].g && top->index != child_index) // if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
            {
                delete child;
                continue;
            }

            child->h = heuristic(child, &goal);

            if (top->index == child->index)
            {
                if (child->cost() > top->cost() + 0.01) { // Constants::tieBreaker
                    delete child;
                    continue;
                } else {
                    child->parent = top->parent;
                }
            }

            child->open();
            nodes3D[child_index] = *child;
            /* child.handle = */ open.push(&nodes3D[child_index]);
            delete child;
        }
    }

    {
        sf::Vertex startAxis[] =
        {
            sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue),
            sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Blue),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue),
            sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Blue),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Blue)
        };

        sf::Vertex goalAxis[] =
        {
            sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green),
            sf::Vertex(sf::Vector2f(0.9, 0.1), sf::Color::Green),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green),
            sf::Vertex(sf::Vector2f(0.9, -0.1), sf::Color::Green),
            sf::Vertex(sf::Vector2f(1, 0), sf::Color::Green)
        };

        ts.push();
        ts.translate(start.x, start.y);
        ts.rotate(start.t * 180 / M_PI);
        window.draw(startAxis, 6, sf::Lines, ts);
        ts.pop();
        ts.push();
        ts.translate(goal.x, goal.y);
        ts.rotate(goal.t * 180 / M_PI);
        window.draw(goalAxis, 6, sf::Lines, ts);
        ts.pop();
    }

    delete[] nodes3D;
    return nullptr;
}

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "MPC", sf::Style::Default, settings);

    sf::Clock clock;

    TransformStack ts;
    ts.rotate(-90);
    ts.scale(1, -1);
    ts.translate(WINDOW_HEIGHT/-2, WINDOW_WIDTH/-2);
    ts.scale(PX_PER_METER, PX_PER_METER);

    Frame globalFrame;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();

        window.clear(sf::Color(50, 50, 50, 255));

        globalFrame.draw(window, ts);
        
        hybrid_a_star(window, ts);
        
        window.display();
    }

    return 0;
}
