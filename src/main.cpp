
#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/visual.hpp"
#include "include/utils.hpp"

int main()
{
    using dyno::visual::transform;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(640, 480), "Planner", sf::Style::Default, settings);

    dyno::visual::TransformStack ts;
    ts.x_up_y_left_centered(window.getSize());

    Eigen::Vector2d mouse_pos_px;

    dyno::GridMap map;
    map.loadPGM("maps/dobson.yaml");
    dyno::visual::GridMapRenderer map_renderer(map);

    while (window.isOpen())
    {
        // Process all window events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) 
            {
                window.close();
                break;
            }
            if (event.type == sf::Event::Resized)
            {
                window.setView(sf::View({0, 0, static_cast<float>(event.size.width), static_cast<float>(event.size.height)}));
                break;
            }
            if (event.type == sf::Event::MouseMoved)
            {
                Eigen::Vector2d event_pos_px(event.mouseMove.x, event.mouseMove.y);

                if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
                {
                    Eigen::Vector2d delta_m = ts.transform_point(event_pos_px) - ts.transform_point(mouse_pos_px);
                    ts.translate(delta_m.x(), delta_m.y());
                }

                mouse_pos_px = event_pos_px;

                break;
            }
            if (event.type == sf::Event::MouseWheelScrolled && event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
            {
                Eigen::Vector2d mouse_pos_m_before = ts.transform_point(mouse_pos_px);

                if (event.mouseWheelScroll.delta > 0)
                {
                    ts.scale(1.1);
                }
                else if (event.mouseWheelScroll.delta < 0)
                {
                    ts.scale(1 / 1.1);
                }

                Eigen::Vector2d mouse_pos_m_after = ts.transform_point(mouse_pos_px);

                Eigen::Vector2d delta_m = mouse_pos_m_after - mouse_pos_m_before;
                ts.translate(delta_m.x(), delta_m.y());

                break;
            }

        }

        // Render new frame
        window.clear(sf::Color(50, 50, 50, 255));

        map_renderer.draw(window, ts);

        transform(ts, { }, [&]() {
            dyno::visual::draw_grid(window, ts, 10);
        });

        transform(ts, { .x=1, .angle=0.0 }, [&]() {
            dyno::visual::draw_frame(window, ts);
        });

        transform(ts, { .x=2, .y=1 }, [&]() {
            dyno::visual::draw_point(window, ts, sf::Color::Cyan, 0.2);
        });

        window.display();

    }

    return 0;
}
