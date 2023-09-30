#include "usv_sim_2d/actuator.hpp"

Eigen::Vector<double, 6> Actuator::propulsion()
{
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
