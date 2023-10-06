#include <iostream>

#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(800, 600), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);

    // view_.reset(sf::FloatRect(0.0, 0.0, 800.0, 600.0));
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
        if (event.type == sf::Event::Closed)
        {
            std::cout << "Closing window, simulation is still running" << '\n';
            window_.close();
        }
        if (event.type == sf::Event::MouseWheelMoved)
        {
            view_ = window_.getDefaultView();
            zoom_ = std::max(zoom_ + 2 * event.mouseWheel.delta, 0.f);
            view_.zoom(1 / zoom_);
            window_.setView(view_);
        }
    }

    // Compute middle of window
    sf::Vector2u size = window_.getSize();
    sf::Vector2u origin_offset = {size.x / 2, size.y / 2};

    if (follow_vessel_)
    {
        view_.setCenter(transform_coord_system(vehice.state.position, origin_offset));
        window_.setView(view_);
    }

    // Draw
    sf::Vector2f position = transform_coord_system(vehice.state.position, origin_offset);
    float heading = vehice.state.attitude.z();

    window_.clear(sf::Color{180, 220, 240});
    static sf::CircleShape shape;

    // Grid
    draw_grid();
    draw_axis(origin_offset);

    // Hull
    static sf::ConvexShape hull(vehice.get_points_of_hull().size());
    size_t i = 0;
    for (Eigen::Vector3d point : vehice.get_points_of_hull())
    {
        sf::Vector2f vertex = transform_coord_system(point, origin_offset);
        hull.setPoint(i, vertex);
        i++;
    }
    hull.setFillColor(sf::Color(252, 174, 30)); // Orange
    window_.draw(hull);

    // Actuators
    shape.setRadius(0.06);
    shape.setFillColor(sf::Color(255, 50, 50)); // Red
    for (Eigen::Vector3d point : vehice.get_points_of_actuators())
    {
        shape.setPosition(get_center_circle(transform_coord_system(point, origin_offset), shape.getRadius()));
        window_.draw(shape);
    }

    // Mass points
    shape.setRadius(0.04);
    shape.setFillColor(sf::Color(255, 255, 50)); // Yellow
    for (Eigen::Vector3d point : vehice.get_points_of_mass())
    {
        shape.setPosition(get_center_circle(transform_coord_system(point, origin_offset), shape.getRadius()));
        window_.draw(shape);
    }
    draw_wake_trail(window_, position, heading);

    // Center of gravity
    sprite_.setPosition(transform_coord_system(vehice.state.position, origin_offset));
    window_.draw(sprite_);

    window_.display();
}

sf::Vector2f Visualisation::get_center_circle(sf::Vector2f corner, float radius)
{
    return {corner.x - radius, corner.y - radius};
}

sf::Vector2f Visualisation::transform_coord_system(const Eigen::Vector3d &position, sf::Vector2u offset)
{
    sf::Vector2f position_transformed = {(float)position.x(), (float)position.y()};
    return {position_transformed.x + offset.x, position_transformed.y + offset.y};
}

sf::Vector2f Visualisation::transform_coord_system(const sf::Vector2f &position, sf::Vector2u offset)
{
    return {position.x + offset.x, position.y + offset.y};
}

void Visualisation::draw_grid()
{
    sf::Vector2u size = window_.getSize();

    // row separators
    static sf::VertexArray grid_row(sf::Lines, 2 * size.x);
    for (size_t i = 0; i < (size.x - 2); i++)
    {
        grid_row[i * 2].position = {0.0, (float)i};
        grid_row[i * 2 + 1].position = {(float)size.x, (float)i};
    }
    window_.draw(grid_row);

    // column separators
    static sf::VertexArray grid_col(sf::Lines, 2 * size.y);
    for (size_t i = 0; i < size.y; i++)
    {
        grid_col[i * 2].position = {(float)i, 0.0};
        grid_col[i * 2 + 1].position = {(float)i, (float)size.y};
    }
    window_.draw(grid_col);
}

void Visualisation::draw_axis(sf::Vector2u origin)
{
    const float line_width = 0.08;

    // X-axis
    sf::RectangleShape axis_x;
    axis_x.setPosition(0.0, origin.y - line_width / 2);
    axis_x.setSize({(float)window_.getSize().x, line_width});
    axis_x.setFillColor(sf::Color{255, 255, 255, 100});
    window_.draw(axis_x);

    // Y-axis
    sf::RectangleShape axis_y;
    axis_y.setPosition(origin.x - line_width / 2, 0.02);
    axis_y.setSize({line_width, (float)window_.getSize().y});
    axis_y.setFillColor(sf::Color{255, 255, 255, 100});
    window_.draw(axis_y);

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

}
