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
    sf::Vector2f origin_offset = {(float)size.x / 2, (float)size.y / 2};

    // Draw
    window_.clear(sf::Color{180, 220, 240});
    draw_grid();

    // Hull
    static sf::ConvexShape hull(vehice.get_points_of_hull().size());
    size_t i = 0;
    for (Eigen::Vector3d point : vehice.get_points_of_hull())
    {
        sf::Vector2f vertex = transform_coord_system(point, origin_offset);
        hull.setPoint(i, vertex);
        i++;
    }
    hull.setFillColor(sf::Color(50, 50, 50));
    window_.draw(hull);

    // Mass points
    static sf::CircleShape shape(5.f);
    shape.setRadius(5.f);
    shape.setFillColor(sf::Color(255, 50, 50));
    for (Eigen::Vector3d point : vehice.get_points_of_mass())
    {
        shape.setPosition(transform_coord_system(point, origin_offset));
        window_.draw(shape);
    }

    // Origin
    shape.setRadius(3.f);
    shape.setFillColor(sf::Color(255, 255, 50));
    shape.setPosition(transform_coord_system(vehice.state.position, zoom_, origin_offset));
    window_.draw(shape);

    window_.display();
}

sf::Vector2f Visualisation::transform_coord_system(const Eigen::Vector3d &position, sf::Vector2u offset)
{
    sf::Vector2f position_transformed = {(float)position.x(), (float)position.y()};

    // Shift origin
    position_transformed = {position_transformed.x + offset.x, position_transformed.y + offset.y};

    return position_transformed;
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
