
#include <iostream>
#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/visual.hpp"
#include "include/utils.hpp"

int main()
{
    std::vector<std::vector<double>> costmap;
    dyno::load_pgm("maps/map2.pgm", costmap);

    sf::Uint8* pixels = new sf::Uint8[costmap.size() * costmap[0].size() * 4];
    for (size_t row = 0; row < costmap.size(); ++row)
    {
        for (size_t col = 0; col < costmap[0].size(); ++col)
        {
            sf::Uint8* pixel = pixels + (row * costmap[0].size() + col) * 4;
            pixel[0] = 255 * costmap[row][col];
            pixel[1] = 255 * costmap[row][col];
            pixel[2] = 255 * costmap[row][col];
            pixel[3] = 255;
        }
    }

    sf::Texture texture;
    if (!texture.create(costmap[0].size(), costmap.size()))
    {
        throw std::runtime_error("Failed to create texture");
    }
    texture.update(pixels);
    texture.setSmooth(false);
    sf::Sprite sprite(texture);

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "Hybrid A*", sf::Style::Default, settings);

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
                // Eigen::Vector2i mouse_grid_index = ((mouse_m - Eigen::Vector2d(-19.390488, -10.627522) ) / 0.050000).cast<int>();
                // double cost = costmap.at(mouse_grid_index.y()).at(mouse_grid_index.x());
                // std::cout << "Mouse grid index: " << mouse_grid_index.x() << ", " << mouse_grid_index.y() << ", " << cost << std::endl;

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

        ts.push();
        ts.translate(-19.390488, -10.627522);
        ts.scale(0.050000);
        window.draw(sprite, ts);
        ts.pop();

        dyno::visual::draw_grid(window, ts, 20);
        dyno::visual::draw_frame(window, ts, 0, 0, 0);

        window.display();
    }

    return 0;
}
