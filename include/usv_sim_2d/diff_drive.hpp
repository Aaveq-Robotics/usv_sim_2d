#pragma once

#include "usv_sim_2d/usv.hpp"

class DiffDrive : public USV
{
public:
    DiffDrive();
    ~DiffDrive() {}

    bool update(std::array<uint16_t, 16> servo_out);

private:
};
