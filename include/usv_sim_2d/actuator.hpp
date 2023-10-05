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
    Actuator(Json::Value properties);
    ~Actuator() {}

    int get_servo_channel() { return servo_channel_; }
    std::string get_type() { return type_; }
    Eigen::Vector3d get_position() { return position_; }
    void set_position(Eigen::Vector3d position) { position_ = position; }

    virtual Eigen::Vector<double, 6> propulsion(const int &servo);

protected:
    int servo_channel_;
    std::string name_;
    std::string type_;
    Eigen::Vector3d position_;
    Eigen::Vector3d attitude_cartesian_;

    double interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);
};