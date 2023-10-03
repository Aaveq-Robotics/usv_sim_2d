#include <iostream>

#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(800, 600), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);
}

// Differential Drive Boat
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
    }

    // Compute middle of window
    sf::Vector2u size = window_.getSize();
    sf::Vector2f origin_offset = {(float)size.x / 2, (float)size.y / 2};

    // Draw
    window_.clear(sf::Color{180, 220, 240});
    draw_grid(10, 10);

    // Origin
    sf::CircleShape shape(5.f);
    shape.setFillColor(sf::Color(255, 255, 50));
    shape.setPosition(transform_coord_system(vehice.state.position));
    window_.draw(shape);

    // Points
    for (Eigen::Vector3d point : vehice.get_point_list_earth())
    {
        shape.setFillColor(sf::Color(255, 50, 50));
        shape.setPosition(transform_coord_system(point));
        window_.draw(shape);
    }

    window_.display();
}

sf::Vector2f Visualisation::transform_coord_system(const Eigen::Vector3d &position)
{
    sf::Vector2f position_transformed = {(float)position.x(), (float)position.y()};

    // Zoom factor
    position_transformed = {position_transformed.x * zoom_, position_transformed.y * zoom_};
    // Shift origin
    position_transformed = {position_transformed.x + origin_offset_.x, position_transformed.y + origin_offset_.y};

    return position_transformed;
}

void Visualisation::draw_grid(int rows, int cols)
{
    /* SOURCE: https://stackoverflow.com/questions/65202726/how-can-i-make-grids-in-sfml-window */

    // initialize values
    int numLines = rows + cols - 2;
    sf::VertexArray grid(sf::Lines, 2 * (numLines));
    window_.setView(window_.getDefaultView());
    auto size = window_.getView().getSize();
    float rowH = size.y / rows;
    float colW = size.x / cols;

    // row separators
    for (int i = 0; i < rows - 1; i++)
    {
        int r = i + 1;
        float rowY = rowH * r;
        grid[i * 2].position = {0, rowY};
        grid[i * 2 + 1].position = {size.x, rowY};
    }

    // column separators
    for (int i = rows - 1; i < numLines; i++)
    {
        int c = i - rows + 2;
        float colX = colW * c;
        grid[i * 2].position = {colX, 0};
        grid[i * 2 + 1].position = {colX, size.y};
    }

    // draw it
    window_.draw(grid);
}
