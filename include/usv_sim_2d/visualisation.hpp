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
    sf::View view_;

    float zoom_ = 100;

    sf::Vector2f transform_coord_system(const Eigen::Vector3d &position, sf::Vector2u offset);
    void draw_grid();
};
