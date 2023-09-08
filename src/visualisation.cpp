#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(800, 600), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);
}

// Differential Drive Boat
void Visualisation::update(DiffDrive &vehice)
{
    // Handle events
    for (auto event = sf::Event{}; window_.pollEvent(event);)
    {
        if (event.type == sf::Event::Closed)
            window_.close();
    }

    // Compute middle of window
    sf::Vector2u size = window_.getSize();
    origin_offset = {(float)size.x / 2, (float)size.y / 2};

    // Draw points
    window_.clear(sf::Color{180, 220, 240});

    for (Eigen::Vector3d point : vehice.point_list)
    {
        sf::CircleShape shape(5.f);
        shape.setFillColor(sf::Color(255, 50, 50));
        shape.setPosition(origin_offset.x + point.x() + vehice.state.position.x(), origin_offset.y + point.y() + vehice.state.position.y());
        window_.draw(shape);
    }

    window_.display();
}
