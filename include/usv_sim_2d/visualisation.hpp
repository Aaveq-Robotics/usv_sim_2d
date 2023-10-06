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
    bool follow_vessel_ = true;

    sf::Texture texture_;
    sf::Sprite sprite_;

    sf::Vector2f get_center_circle(sf::Vector2f corner, float radius);
    sf::Vector2f transform_coord_system(const Eigen::Vector3d &position, sf::Vector2u offset);
    sf::Vector2f transform_coord_system(const sf::Vector2f &position, sf::Vector2u offset);
    void draw_grid();
    void draw_axis(sf::Vector2u origin);
    void draw_wake_trail(sf::RenderWindow &window, const sf::Vector2f &position, const float &heading);
    void draw_actuators(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_actuators, const std::vector<double> &forces_actuators, const sf::Vector2u &offset, const float &heading);
};
