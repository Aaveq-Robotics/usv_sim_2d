#pragma once

#include "usv_sim_2d/actuator.hpp"

class Propeller : public Actuator
{
public:
    Propeller(Json::Value properties);
    ~Propeller() {}

    Eigen::Vector<double, 6> propulsion(const int &servo);

private:
    int servo_idx_;
    Eigen::Vector3d position_;
    Eigen::Vector3d attitude_cartesian_;
    double ang_vel_min_;
    double ang_vel_max_;
    double k_t_;
};