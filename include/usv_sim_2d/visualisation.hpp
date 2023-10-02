#pragma once

#include <SFML/Graphics.hpp>

#include "usv_sim_2d/usv.hpp"

class Visualisation
{
public:
    Visualisation();
    ~Visualisation() {}

    void update(USV &vehice);

private:
    sf::RenderWindow window_;
    sf::Vector2f origin_offset_;

    const int zoom_ = 100;

    void draw_grid(int rows, int cols);
};
