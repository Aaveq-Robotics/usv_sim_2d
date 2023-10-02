#include <iostream>

#include "usv_sim_2d/propeller.hpp"

Propeller::Propeller(Json::Value properties) : Actuator(properties)
{
    ang_vel_min_ = properties["ang_vel_min"].asDouble();
    ang_vel_max_ = properties["ang_vel_max"].asDouble();
    k_t_ = properties["k_t"].asDouble();
}

Eigen::Vector<double, 6> Propeller::propulsion(const int &servo)
{
    Eigen::Vector<double, 6> tau{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double angular_velocity = 0.0;

    // Ensure servo signals are within range
    if (servo != std::clamp(servo, SERVO_MIN, SERVO_MAX))
    {
        std::cout << "[Propeller]: servo was out of range" << '\n';
        return tau;
    }

    // Transform to target range
    angular_velocity = interval_map(servo, SERVO_MIN, SERVO_MAX, -ang_vel_max_, ang_vel_max_);
    angular_velocity = std::clamp(angular_velocity, ang_vel_min_, ang_vel_max_);

    // Compute propulsion forces
    Eigen::Vector3d thrust = attitude_cartesian_ * k_t_ * std::pow(angular_velocity, 2);

    // Compute propulsion moments - the position describes the moment arm
    Eigen::Vector3d torque = position_.cross(thrust);

    // Return forces
    tau << thrust, torque;
    return tau;
}
