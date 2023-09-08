#pragma once

#include <SFML/Graphics.hpp>

class Visualisation
{
public:
    Visualisation();
    ~Visualisation() {}

    void update();

private:
    sf::RenderWindow window_;
    sf::CircleShape shape_;

    sf::Vector2f origin_offset;
};
