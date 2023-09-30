#pragma once

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>

#define SERVO_MIN 1100
#define SERVO_TRIM 1500
#define SERVO_MAX 1900

class Actuator
{
public:
    Actuator() {}
    ~Actuator() {}

    Eigen::Vector<double, 6> propulsion();

protected:
    double interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);
};