
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
    window.setFramerateLimit(120);

    dyno::visual::TransformStack ts;
    ts.x_up_y_left_centered(window.getSize());

    Eigen::Vector2d mouse_pos_px;

    dyno::GridMap occupancy_map;
    occupancy_map.loadPGM("maps/dobson.yaml", true);
    dyno::visual::GridMapRenderer occupancy_map_renderer(occupancy_map, sf::Color::White, sf::Color::Black, 0.3);

    dyno::GridMap esdf_map;
    esdf_map.makeESDF(occupancy_map);
    dyno::visual::GridMapRenderer esdf_map_renderer(esdf_map, sf::Color::Blue, sf::Color::Red);

    dyno::GridMap ridge_map;
    ridge_map.makeRidge(esdf_map, 0.8, 100.0);
    dyno::visual::GridMapRenderer ridge_map_renderer(ridge_map, sf::Color::Transparent, sf::Color::Green, 1.0);

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

        Eigen::Vector2d mouse_pos_m = ts.transform_point(mouse_pos_px);
        double v, dvx, dvy;

        if (occupancy_map.isInside(mouse_pos_m.x(), mouse_pos_m.y())) {
            size_t grid_row, grid_col;
            occupancy_map.worldToGrid(mouse_pos_m.x(), mouse_pos_m.y(), grid_row, grid_col);
            esdf_map.interpolate(mouse_pos_m.x(), mouse_pos_m.y(), v, dvx, dvy, 1000.0);
            printf("Mouse (px): %.1f, %.1f -> (m): %.2f, %.2f -> (grid): %zu, %zu -> value: %.1f\n, esdf -> v: %.2f, dvx: %.2f, dvy: %.2f\n",
                mouse_pos_px.x(), mouse_pos_px.y(),
                mouse_pos_m.x(), mouse_pos_m.y(),
                grid_row, grid_col,
                occupancy_map.at(grid_row, grid_col),
                v, dvx, dvy);
        }

        // Render new frame
        window.clear(sf::Color(50, 50, 50, 255));

        esdf_map_renderer.draw(window, ts);
        occupancy_map_renderer.draw(window, ts);
        ridge_map_renderer.draw(window, ts);

        dyno::visual::draw_grid(window, ts, 10);
        dyno::visual::draw_frame(window, ts);

        transform(ts, { .x=mouse_pos_m.x(), .y=mouse_pos_m.y(), .angle=std::atan2(dvy, dvx) }, [&]() {
            dyno::visual::draw_frame(window, ts);
        });

        transform(ts, { .x=2, .y=1 }, [&]() {
            dyno::visual::draw_point(window, ts, sf::Color::Cyan, 0.2);
        });

        window.display();

    }

    return 0;
}
