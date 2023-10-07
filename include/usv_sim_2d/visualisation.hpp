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

    sf::Vector2f eigen_2_sfml(const Eigen::Vector3d &position);

    void draw_grid(sf::RenderWindow &window);
    void draw_axis(sf::RenderWindow &window);
    void draw_wake_trail(sf::RenderWindow &window, const sf::Vector2f &position, const float &heading);
    void draw_hull(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_hull);
    void draw_actuators(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_actuators, const std::vector<double> &forces_actuators, const float &heading);
    void draw_mass(sf::RenderWindow &window, const std::vector<Eigen::Vector3d> &points_mass);
};
