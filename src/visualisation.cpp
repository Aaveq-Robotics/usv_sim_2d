#include "usv_sim_2d/visualisation.hpp"

Visualisation::Visualisation()
{
    window_.create(sf::VideoMode(800, 600), "USV Simulaiton 2D");
    window_.setFramerateLimit(60);
}

void Visualisation::update()
{
    for (auto event = sf::Event{}; window_.pollEvent(event);)
    {
        if (event.type == sf::Event::Closed)
            window_.close();
    }

    // Compute middle of window
    sf::Vector2u size = window_.getSize();
    origin_offset = {(float)size.x / 2, (float)size.y / 2};

    window_.clear(sf::Color{180, 220, 240});
    window_.display();
}
