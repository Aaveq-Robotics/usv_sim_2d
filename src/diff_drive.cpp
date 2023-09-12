#include "usv_sim_2d/diff_drive.hpp"

DiffDrive::DiffDrive()
{
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
