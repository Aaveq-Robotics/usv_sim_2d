#include "usv_sim_2d/diff_drive.hpp"

DiffDrive::DiffDrive()
{
    point_list_body_.push_back({10.0, 100.0, 0.0, 0.0}); // {m, x, y, z}
    point_list_body_.push_back({15.0, 0.0, 25.0, 0.0});
    point_list_body_.push_back({15.0, 0.0, -25.0, 0.0});

    point_list_body_ = recompute_relative_to_origin(point_list_body_);
    inertia_matrix_ = inertia_matrix(point_list_body_);
}

bool DiffDrive::update(std::array<uint16_t, 16> servo_out)
{
    // Update time
    double timestep = update_timestamp();
    if (timestep < 0.0)
        return false;

    // Rotation
    double max_angle_vel = 0.5;
    double angle_vel = max_angle_vel * timestep;
    state.attitude = state_old.attitude + Eigen::Vector3d{0.0, 0.0, angle_vel};

    // Velocity
    double max_vel = 25.0;
    Eigen::Vector3d vel_body{max_vel * timestep, 0.0, 0.0};
    state.velocity = rotation_matrix_eb(state.attitude) * vel_body;

    // Translation
    state.position = state_old.position + state.velocity;

    // Body to Earth
    point_list_earth_.clear();
    for (auto p : point_list_body_)
    {
        Eigen::Vector3d p_body{p.x, p.y, p.z};
        Eigen::Vector3d p_earth = state.position + rotation_matrix_eb(state.attitude) * p_body;
        point_list_earth_.push_back(p_earth);
    }

    // step the sim forward
    state_old = state;

    // update successful
    return true;
}
