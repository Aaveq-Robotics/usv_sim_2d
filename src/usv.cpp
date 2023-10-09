#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "usv_sim_2d/propeller.hpp"
#include "usv_sim_2d/usv.hpp"

USV::USV()
{
    set_initial_condition({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Init state
    state.timestamp = USV::get_time();
    state.gyro = {0.0, 0.0, 0.0};
    state.accel = {0.0, 0.0, 0.0};
    state.position = {0.0, 0.0, 0.0};
    state.attitude = {0.0, 0.0, 0.0};
    state.velocity = {0.0, 0.0, 0.0};
}

void USV::load_vessel_config(std::string vessel_config_path)
{
    // Load JSON
    std::ifstream vessel_config_file(vessel_config_path, std::ifstream::binary);
    Json::Value vessel_config;
    vessel_config_file >> vessel_config;

    // Load points of mass
    for (auto point : vessel_config["points_of_mass"])
        points_of_mass_.push_back(ADynamics::PointMass({point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble(), point["m"].asDouble()}));

    for (auto point : vessel_config["hull_shape"])
        points_of_hull_.push_back({point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble()});

    mass_ = compute_mass(points_of_mass_);
    origin_ = compute_com(points_of_mass_, mass_);
    points_of_mass_ = recompute_relative_to_origin(points_of_mass_, origin_); // Ensures that the USV position is equal to the origin
    points_of_hull_ = recompute_relative_to_origin(points_of_hull_, origin_); // Ensures that the USV position is equal to the origin
    inertia_matrix_ = ADynamics::inertia_matrix(points_of_mass_);
    mass_matrix_ = ADynamics::mass_matrix(mass_, inertia_matrix_);

    for (auto actuator_config : vessel_config["actuators"])
    {
        Eigen::Vector3d position;
        position[0] = actuator_config["position"]["x"].asDouble();
        position[1] = actuator_config["position"]["y"].asDouble();
        position[2] = actuator_config["position"]["z"].asDouble();

        Actuator *actuator = create_actuator(actuator_config);
        actuator->set_position(recompute_relative_to_origin(position, origin_));
        actuators_.push_back(actuator);
    }
}

Eigen::Vector<double, 6> USV::compute_forces(const std::array<uint16_t, 16> &servo_out)
{
    Eigen::Vector<double, 6> tau{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    forces_of_actuators_.clear();
    for (auto actuator : actuators_)
    {
        Eigen::Vector<double, 6> tau_actuator = actuator->propulsion(servo_out[actuator->get_servo_channel()]);
        forces_of_actuators_.push_back(tau_actuator.head(3).sum() / actuator->get_max_propulsion());
        tau += tau_actuator;
    }

    return tau;
}

bool USV::update_state(const Eigen::Vector<double, 6> &tau)
{
    // Update time
    double timestep = update_timestamp();
    if (timestep < 0.0)
        return false;

    Eigen::Vector<double, 6> state_body_dot;
    Eigen::Vector<double, 6> state_earth_dot;

    // Rigid body dynamics
    std::tie(state_body_, state_body_dot, state_earth_, state_earth_dot) = ADynamics::rigid_body_dynamics(timestep, tau, state_body_, state_earth_, mass_, inertia_matrix_, mass_matrix_);

    // Pass values
    state.gyro = state_body_.tail(3);
    state.accel = state_body_dot.head(3);
    state.position = state_earth_.head(3);
    state.attitude = state_earth_.tail(3);
    state.velocity = state_earth_dot.head(3);

    // Body to Earth
    points_of_mass_earth_.clear();
    for (auto p : points_of_mass_)
    {
        Eigen::Vector3d p_body = p.head(3);
        Eigen::Vector3d p_earth = state.position + ADynamics::rotation_matrix_eb(state.attitude) * p_body;
        points_of_mass_earth_.push_back(p_earth);
    }

    points_of_hull_earth_.clear();
    for (auto p_body : points_of_hull_)
    {
        Eigen::Vector3d p_earth = state.position + ADynamics::rotation_matrix_eb(state.attitude) * p_body;
        points_of_hull_earth_.push_back(p_earth);
    }

    points_of_actuators_earth_.clear();
    for (auto actuator : actuators_)
    {
        Eigen::Vector3d p_earth = state.position + ADynamics::rotation_matrix_eb(state.attitude) * actuator->get_position();
        points_of_actuators_earth_.push_back(p_earth);
    }

    // update successful
    return true;
}

Actuator *USV::create_actuator(Json::Value actuator_config)
{
    std::string type = actuator_config["type"].asString();

    if (type == "propeller")
        return new Propeller(actuator_config);
    else
        return new Actuator;
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
        // The sim is trying to go backwards in time
        std::cout << "[USV] Error: Time went backwards" << std::endl;
        return 0.0;
    }
    else if (timestep == 0.0)
    {
        // Time did not advance. no physics step
        std::cout << "[USV] Warning: Time did not step forward" << std::endl;
        return 0.0;
    }
    else if (timestep > 60)
    {
        // Limiting timestep to less than 1 minute
        std::cout << "[USV] Warning: Time step was very large" << std::endl;
        return 0.0;
    }

    return timestep;
}

void USV::set_initial_condition(const Eigen::Vector<double, 6> &initial_condition)
{
    state_earth_ = initial_condition;
}

double USV::compute_mass(const std::vector<ADynamics::PointMass> &points)
{
    double sum = 0.0;
    for (auto p : points)
        sum += p.m();

    return sum;
}

Eigen::Vector3d USV::compute_com(const std::vector<ADynamics::PointMass> &points, const double &mass)
{
    // Compute center of mass
    Eigen::Vector3d com{0.0, 0.0, 0.0};
    for (auto p : points)
    {
        com[0] += p.m() * p.x();
        com[1] += p.m() * p.y();
        com[2] += p.m() * p.z();
    }

    com[0] /= mass;
    com[1] /= mass;
    com[2] /= mass;

    return com;
}

Eigen::Vector3d USV::recompute_relative_to_origin(const Eigen::Vector3d &point, const Eigen::Vector3d &com)
{
    Eigen::Vector3d point_recomputed = point;

    // Recompute coordinates of points relative to com (origin)
    point_recomputed.x() -= com.x();
    point_recomputed.y() -= com.y();
    point_recomputed.z() -= com.z();

    return point_recomputed;
}

std::vector<Eigen::Vector3d> USV::recompute_relative_to_origin(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &com)
{
    std::vector<Eigen::Vector3d> points_recomputed = points;

    // Recompute coordinates of points relative to com (origin)
    size_t i = 0;
    for (auto point : points_recomputed)
    {
        points_recomputed[i] = recompute_relative_to_origin(point, com);
        i++;
    }

    return points_recomputed;
}

USV::PointMass USV::recompute_relative_to_origin(const USV::PointMass &point, const Eigen::Vector3d &com)
{
    USV::PointMass point_recomputed = point;

    // Recompute coordinates of points relative to com (origin)
    point_recomputed.x -= com.x();
    point_recomputed.y -= com.y();
    point_recomputed.z -= com.z();

    return point_recomputed;
}

std::vector<USV::PointMass> USV::recompute_relative_to_origin(const std::vector<USV::PointMass> &points, const Eigen::Vector3d &com)
{
    std::vector<USV::PointMass> points_recomputed = points;

    // Recompute coordinates of points relative to com (origin)
    size_t i = 0;
    for (auto point : points_recomputed)
    {
        points_recomputed[i] = recompute_relative_to_origin(point, com);
        i++;
    }

    return points_recomputed;
}

