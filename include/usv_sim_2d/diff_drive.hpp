#pragma once

#include "usv_sim_2d/usv.hpp"

class DiffDrive : public USV
{
public:
    DiffDrive();
    ~DiffDrive() {}

    Eigen::Vector<double, 6> compute_forces(const std::array<uint16_t, 16> &servo_out);

private:
};
