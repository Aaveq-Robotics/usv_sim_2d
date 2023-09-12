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

double USV::get_time()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return (double)us / 1000000.0; // Convert to seconds
}

double USV::update_timestamp()
{
    double timestamp_old = state.timestamp;
    state.timestamp = USV::get_time();
    double timestep = state.timestamp - timestamp_old;

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

void USV::set_initial_condition(const Eigen::Vector<double, 6> &initial_condition)
{
    eta_ = initial_condition;
}

double USV::sum_mass(const std::vector<USV::PointMass> &points)
{
    double sum = 0.0;
    for (auto p : points)
        sum += p.m;

    return sum;
}

Eigen::Matrix3d USV::skew_symmetric_matrix(const Eigen::Vector3d &v)
{
    return Eigen::Matrix3d{{0, -v.z(), v.y()}, {v.z(), 0, -v.x()}, {-v.y(), v.x(), 0}};
}

std::vector<USV::PointMass> USV::recompute_relative_to_origin(const std::vector<USV::PointMass> &points)
{
    /* Ensures that the position is equal to the origin / center of mass */

    std::vector<USV::PointMass> points_recomputed;
    points_recomputed = points;

    // Compute center of mass
    PointMass com(0.0, 0.0, 0.0, 0.0);
    for (auto p : points_recomputed)
    {
        com.m += p.m;
        com.x += p.m * p.x;
        com.y += p.m * p.y;
        com.z += p.m * p.z;
    }
    com.x /= com.m;
    com.y /= com.m;
    com.z /= com.m;

    // Recompute coordinates of points relative to com (origin)
    for (size_t i = 0; i < points_recomputed.size(); i++)
    {
        points_recomputed[i].x -= com.x;
        points_recomputed[i].y -= com.y;
        points_recomputed[i].z -= com.z;
    }

    return points_recomputed;
}

Eigen::Matrix3d USV::inertia_matrix(const std::vector<USV::PointMass> &points)
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

Eigen::Matrix<double, 6, 6> USV::mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix)
{
    Eigen::Matrix<double, 6, 6> mass_matrix;
    mass_matrix << mass * Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), inertia_matrix;

    return mass_matrix;
}

Eigen::Matrix<double, 6, 6> USV::coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu)
{
    Eigen::Vector<double, 3> omega = nu.tail(3); // Get last 3 elements
    Eigen::Matrix<double, 6, 6> coriolis_matrix;
    coriolis_matrix << mass * skew_symmetric_matrix(omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), -skew_symmetric_matrix(inertia_matrix * omega);

    return coriolis_matrix;
}

Eigen::Matrix3d USV::rotation_matrix_eb(const Eigen::Vector3d &attitude)
{
    double phi = attitude.x();
    double theta = attitude.y();
    double psi = attitude.z();

    // Rotation of earth-fixed (NED) frame with respect to body-fixed frame
    Eigen::Matrix3d Rx{{1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}};
    Eigen::Matrix3d Ry{{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}};
    Eigen::Matrix3d Rz{{cos(psi), -sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};

    return Rz * Ry * Rx;
}

Eigen::Matrix3d USV::transformation_matrix(const Eigen::Vector3d &attitude)
{
    double phi = attitude.x();
    double theta = attitude.y();

    return Eigen::Matrix3d{{1, sin(phi) * (sin(theta) / cos(theta)), cos(phi) * (sin(theta) / cos(theta))},
                           {0, cos(phi), -sin(phi)},
                           {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}};
}

Eigen::Matrix<double, 6, 6> USV::J_Theta(const Eigen::Vector<double, 6> &eta)
{
    Eigen::Vector<double, 3> Omega = eta.tail(3); // Get last 3 elements
    Eigen::Matrix<double, 6, 6> J_Theta;
    J_Theta << rotation_matrix_eb(Omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), transformation_matrix(Omega);

    return J_Theta;
}

double USV::interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
