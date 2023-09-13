#include "usv_sim_2d/diff_drive.hpp"

DiffDrive::DiffDrive()
{
    set_initial_condition({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    point_list_body_.push_back({10.0, 100.0, 0.0, 0.0}); // {m, x, y, z}
    point_list_body_.push_back({15.0, 0.0, 25.0, 0.0});
    point_list_body_.push_back({15.0, 0.0, -25.0, 0.0});

    mass_ = sum_mass(point_list_body_);
    point_list_body_ = recompute_relative_to_origin(point_list_body_);
    inertia_matrix_ = inertia_matrix(point_list_body_);
    mass_matrix_ = mass_matrix(mass_, inertia_matrix_);
}

Eigen::Vector<double, 6> DiffDrive::compute_forces(const std::array<uint16_t, 16> &servo_out)
{
    return Eigen::Vector<double, 6>{1000.0, 0.0, 0.0, 0.0, 0.0, 10000.0};
}

double DiffDrive::interval_map(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    // x: value
    // [x0, x1]: original interval
    // [y0, y1]: target interval

    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
