#pragma once

#include <Eigen/Dense>
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

    sf::Vector2f transform_coord_system(const Eigen::Vector3d &position);
    void draw_grid(int rows, int cols);
};
