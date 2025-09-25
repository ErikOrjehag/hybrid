
#include <SFML/Graphics.hpp>
#include "include/transform_stack.hpp"
#include "include/visual.hpp"
#include "include/utils.hpp"
#include "include/rs_path.hpp"
#include "include/hybrid_astar.hpp"
#include "vendor/json.hpp"
#include "include/ompl_planner.hpp"
#include <fstream>

nlohmann::json read_json_file(const std::string& filename)
{
    std::ifstream json_file(filename);
    if (!json_file.is_open())
    {
        throw std::runtime_error("Failed to open JSON file: " + filename);
    }
    nlohmann::json j = nlohmann::json::parse(json_file);
    json_file.close();
    return j;
}

int main()
{
    using dyno::visual::transform;
    using dyno::ompl_planner::OMPLPlanner;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(640, 480), "Planner", sf::Style::Default, settings);
    window.setFramerateLimit(120);

    dyno::visual::TransformStack ts;
    ts.x_up_y_left_centered(window.getSize());

    Eigen::Vector2d mouse_pos_px;
    Eigen::Vector2d mouse_pos_m;
    Eigen::Vector2d mouse_pos_m_pressed;

    //
    double from_x = 0.0;
    double from_y = 0.0;
    double from_yaw = 0.0;
    double to_x = 5.0;
    double to_y = 5.0;
    double to_yaw = 0.0;
    //
    auto coverage_plan = read_json_file("maps/dobson/coverage_plan.json");
    auto named_poses = read_json_file("maps/dobson/named_poses.json");
    // printf(coverage_plan.dump(4).c_str());
    //

    dyno::GridMap occupancy_map;
    occupancy_map.loadPGM("maps/dobson/dobson.yaml", true);
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
    std::vector<dyno::rs::Path> rs_paths;
    dyno::rs::Path rs_shortest_path;
    std::vector<dyno::hybrid_a_star::PathNode> astar_path;
    std::vector<dyno::hybrid_a_star::PathNode> astar_frontier;
    dyno::hybrid_a_star::HybridAStarSearch astar_search(
        esdf_map,
        motion_primatives,
        10'000'000,
        0//1'000
    );
    //
    std::vector<std::array<double, 3>> guides;

    const std::string multi_pose_names[] = {
        "charge_enter",
        "charge_exit",
        "dropoff_enter",
        "dropoff_action_in",
        "dropoff_action",
        "dropoff_exit",
    };
    for (const auto& name : multi_pose_names)
    {
        for (const auto& pose : named_poses["named_poses"][name])
        {
            double x = pose["x"];
            double y = pose["y"];
            double yaw = pose["yaw"];
            guides.push_back({x, y, yaw});
        }
    }
    const std::string single_pose_names[] = {
        "charge_action_in",
        "charge_action_pre",
        "charge_action",
    };
    for (const auto& name : single_pose_names)
    {
        const auto& pose = named_poses["named_poses"][name];
        double x = pose["x"];
        double y = pose["y"];
        double yaw = pose["yaw"];
        guides.push_back({x, y, yaw});
    }

    std::vector<std::array<double, 3>> ompl_path;
    OMPLPlanner ompl_planner;
    ompl_planner.setup(esdf_map, guides);
    //

    int coverage_plan_index = 0;

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
                    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                    {
                        from_yaw = mouse_yaw_pressed;
                        printf("%.2f %.2f %.2f\n", from_x, from_y, from_yaw);
                    }
                    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::G))
                    {
                        to_yaw = mouse_yaw_pressed;
                    }
                }

                double v, dvx, dvy;
                if (occupancy_map.isInside(mouse_pos_m.x(), mouse_pos_m.y())) {
                    size_t grid_row, grid_col;
                    occupancy_map.worldToGrid(mouse_pos_m.x(), mouse_pos_m.y(), grid_row, grid_col);
                    esdf_map.interpolate(mouse_pos_m.x(), mouse_pos_m.y(), v, dvx, dvy, 1000.0);
                    esdf_gradient_yaw = std::atan2(dvy, dvx);
                    // printf("Mouse (px): %.1f, %.1f -> (m): %.2f, %.2f -> (grid): %zu, %zu -> value: %.1f\n, esdf -> v: %.2f, dvx: %.2f, dvy: %.2f\n",
                    //     mouse_pos_px.x(), mouse_pos_px.y(),
                    //     mouse_pos_m.x(), mouse_pos_m.y(),
                    //     grid_row, grid_col,
                    //     occupancy_map.at(grid_row, grid_col),
                    //     v, dvx, dvy);
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
                if (event.key.code == sf::Keyboard::O)
                {
                    ompl_planner.plan(from_x, from_y, from_yaw, to_x, to_y, to_yaw, ompl_path);
                }
                if (event.key.code == sf::Keyboard::P)
                {
                    // auto startPose = coverage_plan["coverage_plan"][coverage_plan_index];
                    // double sx = startPose["x"];
                    // double sy = startPose["y"];
                    // double syaw = startPose["yaw"];
                    // astar_search.setStartPose(sx, sy, syaw);
                    // auto goalPose = coverage_plan["coverage_plan"][(coverage_plan_index + 1) % coverage_plan["coverage_plan"].size()];
                    // double gx = goalPose["x"];
                    // double gy = goalPose["y"];
                    // double gyaw = goalPose["yaw"];
                    // astar_search.setGoalPose(gx, gy, gyaw);
                    // coverage_plan_index = (coverage_plan_index + 1) % coverage_plan["coverage_plan"].size();
                    astar_search.setStartPose(from_x, from_y, from_yaw);
                    astar_search.setGoalPose(to_x, to_y, to_yaw);
                    astar_search.start();
                }
                continue;
            }
            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    mouse_pos_m_pressed = mouse_pos_m;
                    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                    {
                        from_x = mouse_pos_m.x();
                        from_y = mouse_pos_m.y();
                    }
                    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::G))
                    {
                        to_x = mouse_pos_m.x();
                        to_y = mouse_pos_m.y();
                    }
                }
                continue;
            }

        }

        if (astar_search.isSearching())
        {
            astar_search.step();
            astar_search.getCurrentPath(astar_path);
            astar_search.getFrontier(astar_frontier, 100);
        } else if (astar_search.foundGoal()) {
            astar_search.getGoalPath(astar_path);
            {
                double start_x, start_y, start_yaw;
                astar_search.getStartPose(start_x, start_y, start_yaw);
                double end_x, end_y, end_yaw;
                astar_search.getGoalPose(end_x, end_y, end_yaw);
                // dyno::rs::shortest_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, 1.0/1.0, 0.1, rs_shortest_path);
            }
        }

        // if (std::hypot(mouse_pos_m.x(), mouse_pos_m.y()) > 0.1) {
        //     dyno::rs::generate_paths(0, 0, 0, mouse_pos_m.x(), mouse_pos_m.y(), esdf_gradient_yaw, 1.0/1.0, 0.1, rs_paths);
        // }

        // dyno::rs::shortest_path(from_x, from_y, from_yaw, to_x, to_y, to_yaw, 1.0/1.0, 0.1, rs_shortest_path);

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
            transform(ts, { .x=start_x, .y=start_y, .angle=start_yaw }, [&]() {
                dyno::visual::draw_frame(window, ts);
            });
            double goal_x, goal_y, goal_yaw;
            astar_search.getStartPose(goal_x, goal_y, goal_yaw);
            transform(ts, { .x=goal_x, .y=goal_y, .angle=goal_yaw }, [&]() {
                dyno::visual::draw_frame(window, ts);
            });
        }

        for (const auto& pose : coverage_plan["coverage_plan"])
        {
            double x = pose["x"];
            double y = pose["y"];
            double yaw = pose["yaw"];
            transform(ts, { .x=x, .y=y, .angle=yaw, .s=0.5 }, [&]() {
                dyno::visual::draw_frame(window, ts);
            });
        }

        for (const auto& path : rs_paths) {
            sf::VertexArray lines(sf::LineStrip, path.x.size());
            for (size_t i = 0; i < path.x.size(); ++i)
            {
                lines[i].position = sf::Vector2f(path.x[i], path.y[i]);
                lines[i].color = sf::Color::Cyan;
            }
            window.draw(lines, ts);
        }

        {
            sf::VertexArray lines(sf::LineStrip, rs_shortest_path.x.size());
            for (size_t i = 0; i < rs_shortest_path.x.size(); ++i)
            {
                lines[i].position = sf::Vector2f(rs_shortest_path.x[i], rs_shortest_path.y[i]);
                lines[i].color = sf::Color::Cyan;
            }
            window.draw(lines, ts);
        }

        {
            const std::string multi_pose_names[] = {
                "charge_enter",
                "charge_exit",
                "dropoff_enter",
                "dropoff_action_in",
                "dropoff_action",
                "dropoff_exit",
            };
            for (const auto& name : multi_pose_names)
            {
                for (const auto& pose : named_poses["named_poses"][name])
                {
                    double x = pose["x"];
                    double y = pose["y"];
                    double yaw = pose["yaw"];
                    transform(ts, { .x=x, .y=y, .angle=yaw, .s=0.5 }, [&]() {
                        dyno::visual::draw_frame(window, ts);
                    });
                }
            }
            const std::string single_pose_names[] = {
                "charge_action_in",
                "charge_action_pre",
                "charge_action",
            };
            for (const auto& name : single_pose_names)
            {
                const auto& pose = named_poses["named_poses"][name];
                double x = pose["x"];
                double y = pose["y"];
                double yaw = pose["yaw"];
                transform(ts, { .x=x, .y=y, .angle=yaw, .s=0.5 }, [&]() {
                    dyno::visual::draw_frame(window, ts);
                });
            }
        }

        if (ompl_path.size() >= 2) {
            sf::VertexArray lines(sf::LineStrip, ompl_path.size());
            for (size_t i = 0; i < ompl_path.size(); ++i)
            {
                lines[i].position = sf::Vector2f(ompl_path[i][0], ompl_path[i][1]);
                lines[i].color = sf::Color::Cyan;

                if (i % 5 == 0) {
                    transform(ts, { .x=ompl_path[i][0], .y=ompl_path[i][1], .angle=ompl_path[i][2], .s=0.1 }, [&]() {
                        dyno::visual::draw_frame(window, ts);
                    });
                }
            }
            window.draw(lines, ts);
        }

        transform(ts, { .x=from_x, .y=from_y, .angle=from_yaw }, [&]() {
            dyno::visual::draw_frame(window, ts);
        });

        transform(ts, { .x=to_x, .y=to_y, .angle=to_yaw }, [&]() {
            dyno::visual::draw_frame(window, ts);
        });

        window.display();

    }

    return 0;
}
