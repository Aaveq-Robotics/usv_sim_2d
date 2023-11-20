#pragma once

#include <chrono>
#include <time.h>
#include <Eigen/Dense>

#include "aaveq_dynamics/adynamics.hpp"

#include "usv_sim_2d/actuator.hpp"

class USV
{
public:
    struct VechicleState
    {
        double timestamp = 0.0;

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d attitude;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d angular_acceleration;
    } state;

    USV();
    ~USV() {}

    void load_vessel_config(std::string vessel_config_path);
    bool update_state(const std::array<uint16_t, 16> &servo_out);

    std::vector<Eigen::Vector3d> get_points_of_mass() { return points_of_mass_earth_; };
    std::vector<Eigen::Vector3d> get_points_of_hull() { return points_of_hull_earth_; };
    std::vector<Eigen::Vector3d> get_points_of_actuators() { return points_of_actuators_earth_; };
    std::vector<double> get_forces_of_actuators() { return forces_of_actuators_; };

private:
    // Member variables
    std::vector<ADynamics::PointMass> points_of_mass_;
    std::vector<Eigen::Vector3d> points_of_mass_earth_;
    std::vector<Eigen::Vector3d> points_of_hull_;
    std::vector<Eigen::Vector3d> points_of_hull_earth_;
    std::vector<Eigen::Vector3d> points_of_actuators_earth_;
    std::vector<Actuator *> actuators_;
    std::vector<double> forces_of_actuators_;
    double mass_;
    double drag_coeff_;
    double hull_depth_;

    Eigen::Vector<double, 6> state_body_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 6> state_earth_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d inertia_matrix_;
    Eigen::Matrix<double, 6, 6> mass_matrix_;

    // Member functions
    Actuator *create_actuator(Json::Value actuator_config);
    Eigen::Vector<double, 6> compute_forces(const std::array<uint16_t, 16> &servo_out);

    double get_time();
    double update_timestamp();
    void set_initial_condition(const Eigen::Vector<double, 6> &initial_condition);
    double compute_mass(const std::vector<ADynamics::PointMass> &points);
    Eigen::Vector3d compute_com(const std::vector<ADynamics::PointMass> &points, const double &mass);

    template <typename EigenVec>
    EigenVec recompute_relative_to_origin(const EigenVec &point, const Eigen::Vector3d &com);
    template <typename EigenVec>
    std::vector<EigenVec> recompute_relative_to_origin(const std::vector<EigenVec> &points, const Eigen::Vector3d &com);
};
