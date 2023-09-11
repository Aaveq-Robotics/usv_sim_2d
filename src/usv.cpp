#include <iostream>

#include "usv_sim_2d/usv.hpp"

USV::USV()
{
    // Init state
    state_old.timestamp = USV::micros();
    state_old.gyro = {0.0, 0.0, 0.0};
    state_old.accel = {0.0, 0.0, 0.0};
    state_old.position = {0.0, 0.0, 0.0};
    state_old.attitude = {0.0, 0.0, 0.0};
    state_old.velocity = {0.0, 0.0, 0.0};
}

bool USV::update(std::array<uint16_t, 16> servo_out)
{
    // ideal vehicle model
    /* servos are defined as
        1. throttle (really just velocity control)
        2. steering (really just turn rate omega)
     */

    // Update time
    double timestep = update_timestamp();
    if (timestep < 0.0)
        return false;

    // how fast is the rover moving
    double max_velocity = 1.0; // m/s
    double body_v = interval_map(servo_out[2], 1100.0, 1900.0, -max_velocity, max_velocity);

    // update the state
    state.velocity[0] = body_v;
    state.accel[0] = (state.velocity[0] - state_old.velocity[0]) / timestep; // derivative for accel
    double delta_pos_x = (state.velocity[0]) * timestep;                     // integrate for position change
    state.position[0] = delta_pos_x + state_old.position[0];                 // plus c

    state.accel[2] = -9.82; // Gravity

    // step the sim forward
    state_old = state;

    // update successful
    return true;
}

Eigen::Matrix3d USV::inertia_matrix(std::vector<USV::PointMass> points)
{
    auto inertia = [](double x, double y, double z) -> Eigen::Matrix3d
    {
        return Eigen::Matrix3d{{std::pow(y, 2) + std::pow(z, 2), -x * y, -x * z},
                               {-x * y, std::pow(x, 2) + std::pow(z, 2), -y * z},
                               {-x * z, -y * z, std::pow(x, 2) + std::pow(y, 2)}};
    };

    Eigen::Matrix3d I;
    for (auto p : points)
        I += p.m * inertia(p.x, p.y, p.z);

    return I;
}

Eigen::Matrix3d USV::rotation_matrix_eb(double phi, double theta, double psi)
{
    // Rotation of earth-fixed (NED) frame with respect to body-fixed frame
    Eigen::Matrix3d Rx{{1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}};
    Eigen::Matrix3d Ry{{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}};
    Eigen::Matrix3d Rz{{cos(psi), -sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};

    return Rz * Ry * Rx;
}

Eigen::Matrix3d transformation_matrix(double phi, double theta)
{
    return Eigen::Matrix3d{{1, sin(phi) * (sin(theta) / cos(theta)), cos(phi) * (sin(theta) / cos(theta))},
                           {0, cos(phi), -sin(phi)},
                           {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}};
}

double USV::micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return (double)us / 1000000.0; // Convert to seconds
}

double USV::update_timestamp()
{
    state.timestamp = USV::micros();
    double timestep = state.timestamp - state_old.timestamp;

    if (timestep < 0.0)
    {
        // the sim is trying to go backwards in time
        std::cout << "[USV] Error: Time went backwards" << std::endl;
        return 0.0;
    }
    else if (timestep == 0.0)
    {
        // time did not advance. no physics step
        std::cout << "[USV] Warning: Time did not step forward" << std::endl;
        return 0.0;
    }
    else if (timestep > 60)
    {
        // limiting timestep to less than 1 minute
        std::cout << "[USV] Warning: Time step was very large" << std::endl;
        return 0.0;
    }

    return timestep;
}

double USV::interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
