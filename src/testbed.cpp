
#include <iostream>
#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/visual.hpp"

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "MPC", sf::Style::Default, settings);

    dyno::visual::TransformStack ts;

    ts.x_up_y_left_centered(window.getSize());

    Eigen::Vector2d mouse_px;

    while (window.isOpen())
    {
    
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) 
            {
                window.close();
            }
            else if (event.type == sf::Event::Resized)
            {
                window.setView(sf::View({0, 0, static_cast<float>(event.size.width), static_cast<float>(event.size.height)}));
            }
            // handle mouse drag event
            else if (event.type == sf::Event::MouseMoved)
            {
                Eigen::Vector2d event_px(event.mouseMove.x, event.mouseMove.y);

                if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
                {
                    Eigen::Vector2d delta_m = ts.transform_point(event_px) - ts.transform_point(mouse_px);
                    ts.translate(delta_m.x(), delta_m.y());
                }

                mouse_px = event_px;

                // Eigen::Vector2d mouse_m = ts.transform_point(mouse_px);
                // std::cout << "Mouse moved: " << mouse_m.x() << ", " << mouse_m.y() << std::endl;
            }
            else if (event.type == sf::Event::MouseWheelScrolled && event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
            {   
                Eigen::Vector2d mouse_m_before = ts.transform_point(mouse_px);

                if (event.mouseWheelScroll.delta > 0)
                {
                    ts.scale(1.1);
                }
                else if (event.mouseWheelScroll.delta < 0)
                {
                    ts.scale(1/1.1);
                }

                Eigen::Vector2d mouse_m_after = ts.transform_point(mouse_px);

                Eigen::Vector2d delta_m = mouse_m_after - mouse_m_before;
                ts.translate(delta_m.x(), delta_m.y());
                
                // std::cout << "Mouse scroll delta: " << event.mouseWheelScroll.delta << std::endl;
            }
        }

        window.clear(sf::Color(50, 50, 50, 255));

        dyno::visual::draw_frame(window, ts, 0, 0, 0);

        window.display();
    }

    // sf::Clock clock;

    // TransformStack ts;
    // ts.rotate(-90);
    // ts.scale(1, -1);
    // ts.translate(5*WINDOW_HEIGHT/-6, 5*WINDOW_WIDTH/-6);
    // ts.scale(PX_PER_METER, PX_PER_METER);

    // Frame globalFrame;

    // while (window.isOpen())
    // {
    //     sf::Event event;
    //     while (window.pollEvent(event))
    //     {
    //         if (event.type == sf::Event::Closed)
    //         {
    //             window.close();
    //         }
    //         else if (event.type == sf::Event::KeyPressed)
    //         {
    //             if (event.key.code == sf::Keyboard::Num1)
    //             {
    //                 controlState = SET_START;
    //             }
    //             else if (event.key.code == sf::Keyboard::Num2)
    //             {
    //                 controlState = SET_GOAL;
    //             }
    //             else if (event.key.code == sf::Keyboard::Up)
    //             {
    //                 max_iter += 1;
    //             }
    //             else if (event.key.code == sf::Keyboard::Down)
    //             {
    //                 max_iter = std::max(0, max_iter - 1);
    //             }
    //         }
    //         else if (event.type == sf::Event::MouseButtonPressed)
    //         {
    //             if (event.mouseButton.button == sf::Mouse::Button::Left)
    //             {
    //                 mousePosPress = sf::Transform(ts).getInverse() * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
    //             }
    //         }
    //         else if (event.type == sf::Event::MouseButtonReleased)
    //         {
    //             if (event.mouseButton.button == sf::Mouse::Button::Left)
    //             {
    //                 sf::Vector2f mousePosRelease = sf::Transform(ts).getInverse() * sf::Vector2f(event.mouseButton.x, event.mouseButton.y);
    //                 sf::Vector2f mousePosDelta = mousePosRelease - mousePosPress;
    //                 float theta = atan2(mousePosDelta.y, mousePosDelta.x);
    //                 if (controlState == SET_GOAL)
    //                 {
    //                     goal.x = mousePosPress.x;
    //                     goal.y = mousePosPress.y;
    //                     goal.z = theta;
    //                 }
    //                 else if (controlState == SET_START)
    //                 {
    //                     start.x = mousePosPress.x;
    //                     start.y = mousePosPress.y;
    //                     start.z = theta;
    //                 }
    //             }
    //         }
    //     }

    //     float dt = clock.restart().asSeconds();

    //     window.clear(sf::Color(50, 50, 50, 255));

    //     globalFrame.draw(window, ts);
        
    //     hybrid_a_star(window, ts, start, goal, path, max_iter);

    //     if (path.size() > 0)
    //     {
    //         sf::Vertex line[path.size()];
    //         for (size_t i = 0; i < path.size(); i++)
    //         {
    //             line[i] = sf::Vertex(sf::Vector2f(path[i].x, path[i].y), sf::Color::Red);
    //         }
    //         window.draw(line, path.size(), sf::LinesStrip, ts);
    //     }

    //     {
    //         sf::Vertex startAxis[] =
    //         {
    //             sf::Vertex(sf::Vector2f(-0.2, 0), sf::Color::Blue),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
    //             sf::Vertex(sf::Vector2f(-0.1, 0.1), sf::Color::Blue),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue),
    //             sf::Vertex(sf::Vector2f(-0.1, -0.1), sf::Color::Blue),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Blue)
    //         };

    //         sf::Vertex goalAxis[] =
    //         {
    //             sf::Vertex(sf::Vector2f(-0.2, 0), sf::Color::Green),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
    //             sf::Vertex(sf::Vector2f(-0.1, 0.1), sf::Color::Green),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green),
    //             sf::Vertex(sf::Vector2f(-0.1, -0.1), sf::Color::Green),
    //             sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green)
    //         };

    //         ts.push();
    //         ts.translate(start.x, start.y);
    //         ts.rotate(start.z * 180 / M_PI);
    //         window.draw(startAxis, 6, sf::Lines, ts);
    //         ts.pop();
    //         ts.push();
    //         ts.translate(goal.x, goal.y);
    //         ts.rotate(goal.z * 180 / M_PI);
    //         window.draw(goalAxis, 6, sf::Lines, ts);
    //         ts.pop();
    //     }
        
    //     window.display();
    // }

    return 0;
}
