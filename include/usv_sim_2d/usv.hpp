#pragma once

#include <chrono>
#include <time.h>
#include <Eigen/Dense>

class USV
{
public:
    struct VechicleState
    {
        double timestamp = 0.0;

        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
        Eigen::Vector3d position;
        Eigen::Vector3d attitude;
        Eigen::Vector3d velocity;
    } state, state_old;

    struct PointMass
    {
        double m;
        double x;
        double y;
        double z;
    };

    std::vector<PointMass> point_list;

    USV();
    ~USV() {}

    bool update(std::array<uint16_t, 16> servo_out);

protected:
    double micros();
    double update_timestamp();

private:
    double interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);
};
