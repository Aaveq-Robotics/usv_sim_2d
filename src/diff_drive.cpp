#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "usv_sim_2d/diff_drive.hpp"

DiffDrive::DiffDrive()
{
    set_initial_condition({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void DiffDrive::load_vessel_config(std::string vessel_config_path)
{
    // Load JSON
    std::ifstream vessel_config_file(vessel_config_path, std::ifstream::binary);
    Json::Value vessel_config;
    vessel_config_file >> vessel_config;

    // Load points of mass
    for (auto point : vessel_config["points_of_mass"])
        point_list_body_.push_back({point["m"].asDouble(), point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble()});

    mass_ = sum_mass(point_list_body_);
    point_list_body_ = recompute_relative_to_origin(point_list_body_);
    inertia_matrix_ = inertia_matrix(point_list_body_);
    mass_matrix_ = mass_matrix(mass_, inertia_matrix_);
}

Eigen::Vector<double, 6> DiffDrive::compute_forces(const std::array<uint16_t, 16> &servo_out)
{
    Eigen::Vector<double, 6> tau{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Compute forces
    Eigen::Vector<double, 6> tau_propulsion = force_propulsion(servo_out[SERVO_THROTTLE_LEFT], servo_out[SERVO_THROTTLE_RIGHT]);

    // Sum forces
    tau = tau_propulsion;

    return tau;
}

double DiffDrive::interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    // x: value
    // [x0, x1]: original interval
    // [y0, y1]: target interval

    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}

Eigen::Vector<double, 6> DiffDrive::force_propulsion(const int &servo_left, const int &servo_right)
{
    Eigen::Vector<double, 6> tau{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double angular_velocity_left = 0.0;
    double angular_velocity_right = 0.0;

    // Ensure servo signals are within range
    if (servo_left != std::clamp(servo_left, SERVO_MIN, SERVO_MAX))
    {
        std::cout << "[DiffDrive]: servo_left was out of range" << '\n';
        return tau;
    }
    if (servo_right != std::clamp(servo_right, SERVO_MIN, SERVO_MAX))
    {
        std::cout << "[DiffDrive]: servo_right was out of range" << '\n';
        return tau;
    }

    // Transform to target range
    if (servo_left > SERVO_TRIM)
        angular_velocity_left = interval_map(servo_left, SERVO_TRIM, SERVO_MAX, ANG_VEL_MIN, ANG_VEL_MAX);

    if (servo_right > SERVO_TRIM)
        angular_velocity_right = interval_map(servo_right, SERVO_TRIM, SERVO_MAX, ANG_VEL_MIN, ANG_VEL_MAX);

    // Compute propulsion forces
    double thrust_left = K_T * std::pow(angular_velocity_left, 2);
    double thrust_right = K_T * std::pow(angular_velocity_right, 2);

    // Compute propulsion moments
    double moment_arm_left = MOMENT_ARM_LEN_LEFT;
    double moment_arm_right = MOMENT_ARM_LEN_RIGHT;
    double torque_left = thrust_left * moment_arm_left;
    double torque_right = thrust_right * moment_arm_right;

    // Sum forces
    tau += Eigen::Vector<double, 6>{thrust_left + thrust_right, 0.0, 0.0, 0.0, 0.0, torque_left + torque_right};

    return tau;
}
