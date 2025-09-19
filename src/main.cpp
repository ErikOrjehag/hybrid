
#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/visual.hpp"
#include "include/utils.hpp"
#include "include/rs_path.hpp"
#include "include/hybrid_astar.hpp"

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
    Eigen::Vector2d mouse_pos_m;
    Eigen::Vector2d mouse_pos_m_pressed;

    dyno::GridMap occupancy_map;
    occupancy_map.loadPGM("maps/dobson.yaml", true);
    dyno::visual::GridMapRenderer occupancy_map_renderer(occupancy_map, sf::Color::White, sf::Color::Black, 0.3);

    dyno::GridMap esdf_map;
    esdf_map.makeESDF(occupancy_map);
    dyno::visual::GridMapRenderer esdf_map_renderer(esdf_map, sf::Color::Blue, sf::Color::Red);

    dyno::GridMap ridge_map;
    ridge_map.makeRidge(esdf_map, 0.8, 100.0);
    dyno::visual::GridMapRenderer ridge_map_renderer(ridge_map, sf::Color::Transparent, sf::Color::Green, 1.0);

    //
    double esdf_gradient_yaw = 0.0;
    dyno::hybrid_a_star::MotionPrimitives motion_primatives = dyno::hybrid_a_star::precompute_motion_primatives();
    std::vector<dyno::rs::Path> paths;
    std::vector<dyno::hybrid_a_star::PathNode> astar_path;
    std::vector<dyno::hybrid_a_star::PathNode> astar_frontier;
    dyno::hybrid_a_star::HybridAStarSearch astar_search(
        esdf_map,
        motion_primatives,
        10'000'000,
        1'000
    );
    //

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
                continue;
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
                mouse_pos_m = ts.transform_point(mouse_pos_px);

                if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
                {
                    Eigen::Vector2d delta_m = mouse_pos_m - mouse_pos_m_pressed;
                    double mouse_yaw_pressed = std::atan2(delta_m.y(), delta_m.x());
                    astar_search.setStartPose(mouse_pos_m_pressed.x(), mouse_pos_m_pressed.y(), mouse_yaw_pressed);
                }

                double v, dvx, dvy;
                if (occupancy_map.isInside(mouse_pos_m.x(), mouse_pos_m.y())) {
                    size_t grid_row, grid_col;
                    occupancy_map.worldToGrid(mouse_pos_m.x(), mouse_pos_m.y(), grid_row, grid_col);
                    esdf_map.interpolate(mouse_pos_m.x(), mouse_pos_m.y(), v, dvx, dvy, 1000.0);
                    esdf_gradient_yaw = std::atan2(dvy, dvx);
                    printf("Mouse (px): %.1f, %.1f -> (m): %.2f, %.2f -> (grid): %zu, %zu -> value: %.1f\n, esdf -> v: %.2f, dvx: %.2f, dvy: %.2f\n",
                        mouse_pos_px.x(), mouse_pos_px.y(),
                        mouse_pos_m.x(), mouse_pos_m.y(),
                        grid_row, grid_col,
                        occupancy_map.at(grid_row, grid_col),
                        v, dvx, dvy);
                }

                continue;
            }
            if (event.type == sf::Event::MouseWheelScrolled && event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
            {
                Eigen::Vector2d mouse_pos_m_before = mouse_pos_m;

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

                mouse_pos_m = ts.transform_point(mouse_pos_px); // should not be necessaryp
                continue;
            }
            if (event.type == sf::Event::KeyReleased)
            {
                if (event.key.code == sf::Keyboard::P)
                {
                    astar_search.start(
                        mouse_pos_m.x(), mouse_pos_m.y(), esdf_gradient_yaw
                    );
                }
                continue;
            }
            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    mouse_pos_m_pressed = mouse_pos_m;
                }
                continue;
            }

        }

        if (astar_search.active())
        {
            astar_search.step();
            astar_search.getCurrentPath(astar_path);
            astar_search.getFrontier(astar_frontier, 100);
        }

        if (std::hypot(mouse_pos_m.x(), mouse_pos_m.y()) > 0.1) {
            dyno::rs::generate_paths(0, 0, 0, mouse_pos_m.x(), mouse_pos_m.y(), esdf_gradient_yaw, 1.0/1.0, 0.1, paths);
        }

        // Render new frame
        window.clear(sf::Color(50, 50, 50, 255));

        esdf_map_renderer.draw(window, ts);
        occupancy_map_renderer.draw(window, ts);
        ridge_map_renderer.draw(window, ts);

        dyno::visual::draw_grid(window, ts, 10);
        dyno::visual::draw_frame(window, ts);

        transform(ts, { .x=mouse_pos_m.x(), .y=mouse_pos_m.y(), .angle=esdf_gradient_yaw }, [&]() {
            dyno::visual::draw_frame(window, ts);
        });

        transform(ts, { .x=2, .y=1 }, [&]() {
            dyno::visual::draw_point(window, ts, sf::Color::Cyan, 0.2);
        });

        dyno::visual::draw_motion_primatives_at(window, ts, motion_primatives, 0, 0, 0, 1);

        for (const auto& path : paths) {
            sf::VertexArray lines(sf::LineStrip, path.x.size());
            for (size_t i = 0; i < path.x.size(); ++i)
            {
                lines[i].position = sf::Vector2f(path.x[i], path.y[i]);
                lines[i].color = sf::Color::Cyan;
            }
            window.draw(lines, ts);
        }

        if (astar_path.size() >= 2) {
            sf::VertexArray lines(sf::LineStrip, astar_path.size());
            for (size_t i = 0; i < astar_path.size(); ++i)
            {
                lines[i].position = sf::Vector2f(astar_path[i].x, astar_path[i].y);
                lines[i].color = sf::Color::Yellow;

                if (i % 1 == 0) {
                    transform(ts, { .x=astar_path[i].x, .y=astar_path[i].y, .angle=astar_path[i].yaw, .s=0.1 }, [&]() {
                        dyno::visual::draw_frame(window, ts);
                    });
                }
            }
            window.draw(lines, ts);
        }

        for (const auto& node : astar_frontier)
        {
            transform(ts, { .x=node.x, .y=node.y, .angle=node.yaw, .s=0.1 }, [&]() {
                dyno::visual::draw_frame(window, ts);
            });
        }

        {
            double start_x, start_y, start_yaw;
            astar_search.getStartPose(start_x, start_y, start_yaw);
            transform(ts, { .x=start_x, .y=start_y, .angle=start_yaw, .sx=0.5 }, [&]() {
                dyno::visual::draw_frame(window, ts);
            });
        }

        window.display();

    }

    return 0;
}
