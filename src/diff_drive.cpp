#include "usv_sim_2d/diff_drive.hpp"

DiffDrive::DiffDrive()
{
    point_list_.push_back({10.0, 100.0, 0.0, 0.0}); // {m, x, y, z}
}

bool DiffDrive::update(std::array<uint16_t, 16> servo_out)
{
    // Update time
    double timestep = update_timestamp();
    if (timestep < 0.0)
        return false;

    // update the state
    double max_vel = 50.0;
    double vel = max_vel * timestep;
    state.position = state_old.position + Eigen::Vector3d{vel, vel, 0.0};

    // step the sim forward
    state_old = state;

    // update successful
    return true;
}
