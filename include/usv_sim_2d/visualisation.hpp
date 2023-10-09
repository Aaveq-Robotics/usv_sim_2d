#pragma once

#include <SFML/Graphics.hpp>

#include "usv_sim_2d/diff_drive.hpp"

class Visualisation
{
public:
    Visualisation();
    ~Visualisation() {}

    void update(DiffDrive &vehice);

private:
    sf::RenderWindow window_;
    sf::Vector2f origin_offset;

    void draw_grid(int rows, int cols);
};
