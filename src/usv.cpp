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
        points_of_mass_.push_back({point["m"].asDouble(), point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble()});

    for (auto point : vessel_config["hull_shape"])
        points_of_hull_.push_back({point["x"].asDouble(), point["y"].asDouble(), point["z"].asDouble()});

    mass_ = compute_mass(points_of_mass_);
    origin_ = compute_com(points_of_mass_, mass_);
    points_of_mass_ = recompute_relative_to_origin(points_of_mass_, origin_); // Ensures that the USV position is equal to the origin
    points_of_hull_ = recompute_relative_to_origin(points_of_hull_, origin_); // Ensures that the USV position is equal to the origin
    inertia_matrix_ = inertia_matrix(points_of_mass_);
    mass_matrix_ = mass_matrix(mass_, inertia_matrix_);

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

    for (auto actuator : actuators_)
        tau += actuator->propulsion(servo_out[actuator->get_servo_channel()]);

    return tau;
}

bool USV::rigid_body_dynamics(const Eigen::Vector<double, 6> &tau)
{
    // Update time
    double timestep = update_timestamp();
    if (timestep < 0.0)
        return false;

    // Rigid body dynamics
    Eigen::Vector<double, 6> nu_dot = matrix_inverse(mass_matrix_) * tau - matrix_inverse(mass_matrix_) * coriolis_matrix(mass_, inertia_matrix_, nu_) * nu_;
    nu_dot *= timestep;
    nu_ += nu_dot;

    // Body-fixed frame to earth-fixed frame
    Eigen::Vector<double, 6> eta_dot = J_Theta(eta_) * nu_;
    eta_dot *= timestep;
    eta_ += eta_dot;

    // Pass values
    state.gyro = nu_.tail(3);
    state.accel = nu_dot.head(3);
    state.position = eta_.head(3);
    state.attitude = eta_.tail(3);
    state.velocity = eta_dot.head(3);

    // Body to Earth
    points_of_mass_earth_.clear();
    for (auto p : points_of_mass_)
    {
        Eigen::Vector3d p_body{p.x, p.y, p.z};
        Eigen::Vector3d p_earth = state.position + rotation_matrix_eb(state.attitude) * p_body;
        points_of_mass_earth_.push_back(p_earth);
    }

    points_of_hull_earth_.clear();
    for (auto p_body : points_of_hull_)
    {
        Eigen::Vector3d p_earth = state.position + rotation_matrix_eb(state.attitude) * p_body;
        points_of_hull_earth_.push_back(p_earth);
    }

    points_of_actuators_earth_.clear();
    for (auto actuator : actuators_)
    {
        Eigen::Vector3d p_earth = state.position + rotation_matrix_eb(state.attitude) * actuator->get_position();
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
    eta_ = initial_condition;
}

double USV::compute_mass(const std::vector<USV::PointMass> &points)
{
    double sum = 0.0;
    for (auto p : points)
        sum += p.m;

    return sum;
}
Eigen::Vector3d USV::compute_com(const std::vector<USV::PointMass> &points, const double &mass)
{
    // Compute center of mass
    Eigen::Vector3d com{0.0, 0.0, 0.0};
    for (auto p : points)
    {
        com[0] += p.m * p.x;
        com[1] += p.m * p.y;
        com[2] += p.m * p.z;
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

Eigen::Matrix3d USV::skew_symmetric_matrix(const Eigen::Vector3d &v)
{
    return Eigen::Matrix3d{{0, -v.z(), v.y()}, {v.z(), 0, -v.x()}, {-v.y(), v.x(), 0}};
}

Eigen::Matrix<double, 6, 6> USV::matrix_inverse(const Eigen::Matrix<double, 6, 6> &matrix)
{
    const float epsilon = 1e-6f; // Small tolerance value

    if (matrix.determinant() > epsilon)
        return matrix.inverse();
    else
        return matrix.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::Matrix3d USV::inertia_matrix(const std::vector<USV::PointMass> &points)
{
    auto inertia = [](double x, double y, double z) -> Eigen::Matrix3d
    {
        return Eigen::Matrix3d{{std::pow(y, 2) + std::pow(z, 2), -x * y, -x * z},
                               {-x * y, std::pow(x, 2) + std::pow(z, 2), -y * z},
                               {-x * z, -y * z, std::pow(x, 2) + std::pow(y, 2)}};
    };

    Eigen::Matrix3d I;
    for (auto p : points)
        I += p.m * inertia(p.x, p.y, p.z);

    return I;
}

Eigen::Matrix<double, 6, 6> USV::mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix)
{
    Eigen::Matrix<double, 6, 6> mass_matrix;
    mass_matrix << mass * Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), inertia_matrix;

    return mass_matrix;
}

Eigen::Matrix<double, 6, 6> USV::coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu)
{
    Eigen::Vector<double, 3> omega = nu.tail(3); // Get last 3 elements
    Eigen::Matrix<double, 6, 6> coriolis_matrix;
    coriolis_matrix << mass * skew_symmetric_matrix(omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), -skew_symmetric_matrix(inertia_matrix * omega);

    return coriolis_matrix;
}

Eigen::Matrix3d USV::rotation_matrix_eb(const Eigen::Vector3d &attitude)
{
    double phi = attitude.x();
    double theta = attitude.y();
    double psi = attitude.z();

    // Rotation of earth-fixed (NED) frame with respect to body-fixed frame
    Eigen::Matrix3d Rx{{1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}};
    Eigen::Matrix3d Ry{{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}};
    Eigen::Matrix3d Rz{{cos(psi), -sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};

    return Rz * Ry * Rx;
}

Eigen::Matrix3d USV::transformation_matrix(const Eigen::Vector3d &attitude)
{
    double phi = attitude.x();
    double theta = attitude.y();

    return Eigen::Matrix3d{{1, sin(phi) * (sin(theta) / cos(theta)), cos(phi) * (sin(theta) / cos(theta))},
                           {0, cos(phi), -sin(phi)},
                           {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}};
}

Eigen::Matrix<double, 6, 6> USV::J_Theta(const Eigen::Vector<double, 6> &eta)
{
    Eigen::Vector<double, 3> Omega = eta.tail(3); // Get last 3 elements
    Eigen::Matrix<double, 6, 6> J_Theta;
    J_Theta << rotation_matrix_eb(Omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), transformation_matrix(Omega);

    return J_Theta;
}
