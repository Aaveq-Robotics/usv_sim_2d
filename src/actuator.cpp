#include <iostream>

#include "usv_sim_2d/actuator.hpp"

Actuator::Actuator(Json::Value properties)
{
    name_ = properties["name"].asString();
    type_ = properties["type"].asString();
    std::cout << "Actuator added: " << name_ << " of type " << type_ << '\n';

    servo_channel_ = properties["servo_channel"].asInt();
    position_[0] = properties["position"]["x"].asDouble();
    position_[1] = properties["position"]["y"].asDouble();
    position_[2] = properties["position"]["z"].asDouble();

    // Compute attitude in cartesian coordinates
    double pitch = properties["attitude"]["pitch"].asDouble();
    double yaw = properties["attitude"]["yaw"].asDouble();
    attitude_cartesian_[0] = cos(pitch) * cos(yaw);
    attitude_cartesian_[1] = cos(pitch) * sin(yaw);
    attitude_cartesian_[2] = sin(pitch);
}

Eigen::Vector<double, 6> Actuator::propulsion(const int &servo)
{
    std::cout << "[Actuator] Servo sent to non-specified actuator: " << servo << '\n';

    return Eigen::Vector<double, 6>{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    };
}

double Actuator::interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    // x: value
    // [x0, x1]: original interval
    // [y0, y1]: target interval

    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
