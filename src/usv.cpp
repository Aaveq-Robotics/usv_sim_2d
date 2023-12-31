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

    state.timestamp = USV::micros(); // TODO: Timestamp needs to be double!!
    double timestep = state.timestamp - state_old.timestamp;

    if (timestep < 0)
    {
        // the sim is trying to go backwards in time
        std::cout << "[USV] Error: Time went backwards" << std::endl;
        return false;
    }
    else if (timestep == 0)
    {
        // time did not advance. no physics step
        std::cout << "[USV] Warning: Time did not step forward" << std::endl;
        return false;
    }
    else if (timestep > 60)
    {
        // limiting timestep to less than 1 minute
        std::cout << "[USV] Warning: Time step was very large" << std::endl;
        return false;
    }

    // how fast is the rover moving
    double max_velocity = 1.0; // m/s
    double body_v = _interp1D(servo_out[2], 1100.0, 1900.0, -max_velocity, max_velocity);

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

uint64_t USV::micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return (double)us / 1000000.0;
}

double USV::_interp1D(const double &x, const double &x0, const double &x1, const double &y0, const double &y1)
{
    return ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0);
}
