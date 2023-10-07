#include <iostream>

#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(1000, 1000), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);

    view_ = window_.getDefaultView();
    view_.zoom(1 / zoom_);
    window_.setView(view_);

    // Center of gravity sprite
    texture_.loadFromFile("src/usv_sim_2d/images/center_of_gravity.png");
    sprite_.setTexture(texture_);
    sprite_.setOrigin(texture_.getSize().x / 2, texture_.getSize().y / 2);
    const float desired_size = 0.2;
    sprite_.setScale(sf::Vector2f(desired_size / texture_.getSize().x, desired_size / texture_.getSize().y));
}

void Visualisation::update(USV &vehice)
{
    // Handle events
    for (auto event = sf::Event{}; window_.pollEvent(event);)
    {
        switch (event.type)
        {
        case sf::Event::Closed:
            std::cout << "Closing window, simulation is still running" << '\n';
            window_.close();
            break;

        case sf::Event::Resized:
            view_.setSize({static_cast<float>(event.size.width),
                           static_cast<float>(event.size.height)});
            view_.zoom(1 / zoom_);
            window_.setView(view_);
            break;

        case sf::Event::MouseWheelMoved:
            view_ = window_.getDefaultView();
            zoom_ = std::max(zoom_ + 2 * event.mouseWheel.delta, 0.f);
            view_.zoom(1 / zoom_);
            window_.setView(view_);
            break;

        default:
            break;
        }
    }

    // Manage view
    if (follow_vessel_)
    {
        view_.setCenter(eigen_2_sfml(vehice.state.position));
        window_.setView(view_);
    }

    // Draw
    sf::Vector2f position = eigen_2_sfml(vehice.state.position);
    float heading = vehice.state.attitude.z();

    window_.clear(sf::Color{180, 220, 240});
    draw_grid(window_, {1000, 1000});
    draw_axis(window_, {1000, 1000}, 0.08);
    draw_wake_trail(window_, position, heading);
    draw_hull(window_, vehice.get_points_of_hull());
    draw_actuators(window_, vehice.get_points_of_actuators(), vehice.get_forces_of_actuators(), heading);
    draw_mass(window_, vehice.get_points_of_mass());

    // Center of gravity
    sprite_.setPosition(eigen_2_sfml(vehice.state.position));
    window_.draw(sprite_);

    window_.display();
}

sf::Vector2f Visualisation::eigen_2_sfml(const Eigen::Vector3d &position)
{
    return {(float)position.x(), (float)position.y()};
}

void Visualisation::draw_grid(sf::RenderWindow &window, sf::Vector2u grid_size)
{
    // Size has to be even numbers to align grid with origin
    grid_size.x += grid_size.x % 2;
    grid_size.y += grid_size.y % 2;

    // row separators
    sf::VertexArray grid_row(sf::Lines, 2 * grid_size.x + 2);
    for (size_t i = 0; i <= (grid_size.x); i++)
    {
        grid_row[i * 2].position = {-(float)grid_size.x / 2, (float)i - (float)grid_size.y / 2};
        grid_row[i * 2 + 1].position = {(float)grid_size.x / 2, (float)i - (float)grid_size.y / 2};
    }
    window.draw(grid_row);

    // column separators
    sf::VertexArray grid_col(sf::Lines, 2 * grid_size.y + 2);
    for (size_t i = 0; i <= grid_size.y; i++)
    {
        grid_col[i * 2].position = {(float)i - (float)grid_size.x / 2, -(float)grid_size.y / 2};
        grid_col[i * 2 + 1].position = {(float)i - (float)grid_size.x / 2, (float)grid_size.y / 2};
    }
    window.draw(grid_col);
}

void Visualisation::draw_axis(sf::RenderWindow &window, sf::Vector2u grid_size, const float &line_width)
{
    // Size has to be even numbers to align grid with origin
    grid_size.x += grid_size.x % 2;
    grid_size.y += grid_size.y % 2;

    // X-axis
    sf::RectangleShape axis_x;
    axis_x.setPosition(-(float)grid_size.x / 2, -line_width / 2);
    axis_x.setSize({(float)grid_size.x, line_width});
    axis_x.setFillColor(sf::Color{255, 255, 255, 100});
    window.draw(axis_x);

    // Y-axis
    sf::RectangleShape axis_y;
    axis_y.setPosition(-line_width / 2, -(float)grid_size.y / 2);
    axis_y.setSize({line_width, (float)grid_size.y});
    axis_y.setFillColor(sf::Color{255, 255, 255, 100});
    window.draw(axis_y);
}

void Visualisation::draw_wake_trail(sf::RenderWindow &window, const sf::Vector2f &position, const float &heading)
{
    const float line_width = 0.2;

    static std::vector<sf::Vertex> wake_trail;
    sf::Vertex vertex_0({position.x + ((float)cos(heading + M_PI_2)) * line_width / 2, position.y + ((float)sin(heading + M_PI_2)) * line_width / 2}, sf::Color(255, 255, 255, 100));
    wake_trail.push_back(vertex_0);
    sf::Vertex vertex_1({position.x + ((float)cos(heading - M_PI_2)) * line_width / 2, position.y + ((float)sin(heading - M_PI_2)) * line_width / 2}, sf::Color(255, 255, 255, 100));
    wake_trail.push_back(vertex_1);

    if (wake_trail.size() > 100000) // No more than 100,000 vertices in the wake trail
    {
        // Delete the first two verticies
        wake_trail.erase(begin(wake_trail));
        wake_trail.erase(begin(wake_trail));
    }

    window.draw(&wake_trail[0], wake_trail.size(), sf::TriangleStrip);
}

void Visualisation::draw_hull(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_hull)
{
    static sf::ConvexShape hull(points_hull.size());
    size_t i = 0;
    for (Eigen::Vector3d point : points_hull)
    {
        hull.setPoint(i, eigen_2_sfml(point));
        i++;
    }
    hull.setFillColor(sf::Color(252, 174, 30)); // Orange
    window.draw(hull);
}

void Visualisation::draw_actuators(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_actuators, const std::vector<double> &forces_actuators, const float &heading)
{
    const float radius = 0.06;

    std::vector<sf::Vertex> force_trail;
    static sf::CircleShape shape;
    shape.setRadius(radius);
    shape.setOrigin(radius, radius);
    shape.setFillColor(sf::Color(255, 50, 50)); // Red

    for (size_t i = 0; i < points_actuators.size(); i++)
    //     for (Eigen::Vector3d point : points_actuators)
    {
        sf::Vector2f position = eigen_2_sfml(points_actuators[i]);
        float force = forces_actuators[i] * (6 * radius); // Set max length to 6*r

        // Draw propulsion
        sf::Vertex vertex_0({position.x + ((float)cos(heading + M_PI)) * force, position.y + ((float)sin(heading + M_PI)) * force}, sf::Color(255, 50, 50, 100));
        force_trail.push_back(vertex_0);
        sf::Vertex vertex_1({position.x + ((float)cos(heading + M_PI_2)) * radius, position.y + ((float)sin(heading + M_PI_2)) * radius}, sf::Color(255, 50, 50, 100));
        force_trail.push_back(vertex_1);
        sf::Vertex vertex_2({position.x + ((float)cos(heading - M_PI_2)) * radius, position.y + ((float)sin(heading - M_PI_2)) * radius}, sf::Color(255, 50, 50, 100));
        force_trail.push_back(vertex_2);

        // Draw points
        shape.setPosition(position);
        window.draw(shape);
    }
    window.draw(&force_trail[0], force_trail.size(), sf::Triangles);
}

void Visualisation::draw_mass(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_mass)
{
    const float radius = 0.04;

    static sf::CircleShape shape;
    shape.setRadius(radius);
    shape.setOrigin(radius, radius);

    shape.setFillColor(sf::Color(255, 255, 50)); // Yellow
    for (Eigen::Vector3d point : points_mass)
    {
        shape.setPosition(eigen_2_sfml(point));
        window.draw(shape);
    }
}
