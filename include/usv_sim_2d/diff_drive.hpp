#pragma once

#include "usv_sim_2d/usv.hpp"

#define SERVO_THROTTLE_LEFT 0
#define SERVO_THROTTLE_RIGHT 2
#define ANG_VEL_MIN 0

// Temps
#define ANG_VEL_MAX 1000         // Determined by motor
#define K_T 0.0000001            // Determined by motor
#define MOMENT_ARM_LEN_LEFT -0.1 // Determined by motor placement
#define MOMENT_ARM_LEN_RIGHT 0.1 // Determined by motor placement

class DiffDrive : public USV
{
public:
    DiffDrive();
    ~DiffDrive() {}

    void load_vessel_config(std::string vessel_config_path);

    Eigen::Vector<double, 6> compute_forces(const std::array<uint16_t, 16> &servo_out);

private:
    double interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);

    Eigen::Vector<double, 6> force_propulsion(const int &servo_left, const int &servo_right);
};
