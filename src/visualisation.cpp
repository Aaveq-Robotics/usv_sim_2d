#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(800, 600), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);

    shape_.setRadius(50.f);
    shape_.setFillColor(sf::Color(255, 250, 50));
}

void Visualisation::update()
{
    for (auto event = sf::Event{}; window_.pollEvent(event);)
    {
        if (event.type == sf::Event::Closed)
            window_.close();
    }

    sf::Vector2f pos = shape_.getPosition();
    shape_.setPosition(pos.x + 1.0, pos.y + 1.0);

    window_.clear(sf::Color{180, 220, 240});
    window_.draw(shape_);
    window_.display();
}