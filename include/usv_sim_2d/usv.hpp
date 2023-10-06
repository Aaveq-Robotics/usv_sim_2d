#pragma once

#include <chrono>
#include <time.h>
#include <Eigen/Dense>

#include "usv_sim_2d/actuator.hpp"

class USV
{
public:
    struct VechicleState
    {
        double timestamp = 0.0;

        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
        Eigen::Vector3d position;
        Eigen::Vector3d attitude;
        Eigen::Vector3d velocity;
    } state;

    struct PointMass
    {
        double m;
        double x;
        double y;
        double z;
    };

    USV();
    ~USV() {}

    void load_vessel_config(std::string vessel_config_path);
    Eigen::Vector<double, 6> compute_forces(const std::array<uint16_t, 16> &servo_out);
    bool rigid_body_dynamics(const Eigen::Vector<double, 6> &tau);

    std::vector<Eigen::Vector3d> get_points_of_mass() { return points_of_mass_earth_; };
    std::vector<Eigen::Vector3d> get_points_of_hull() { return points_of_hull_earth_; };
    std::vector<Eigen::Vector3d> get_points_of_actuators() { return points_of_actuators_earth_; };
    std::vector<double> get_forces_of_actuators() { return forces_of_actuators_; };

private:
    // Member variables
    std::vector<PointMass> points_of_mass_;
    std::vector<Eigen::Vector3d> points_of_mass_earth_;
    std::vector<Eigen::Vector3d> points_of_hull_;
    std::vector<Eigen::Vector3d> points_of_hull_earth_;
    std::vector<Eigen::Vector3d> points_of_actuators_earth_;
    std::vector<double> forces_of_actuators_;
    Eigen::Vector3d origin_;
    double mass_;

    std::vector<Actuator *> actuators_;

    Eigen::Vector<double, 6> nu_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 6> eta_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::Matrix3d inertia_matrix_;
    Eigen::Matrix<double, 6, 6> mass_matrix_;

    // Member functions
    Actuator *create_actuator(Json::Value actuator_config);

    double get_time();
    double update_timestamp();
    void set_initial_condition(const Eigen::Vector<double, 6> &initial_condition);
    double compute_mass(const std::vector<PointMass> &points);
    Eigen::Vector3d compute_com(const std::vector<PointMass> &points, const double &mass);
    Eigen::Vector3d recompute_relative_to_origin(const Eigen::Vector3d &point, const Eigen::Vector3d &com);
    std::vector<Eigen::Vector3d> recompute_relative_to_origin(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &com);
    PointMass recompute_relative_to_origin(const PointMass &point, const Eigen::Vector3d &com);
    std::vector<PointMass> recompute_relative_to_origin(const std::vector<PointMass> &points, const Eigen::Vector3d &com);
    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);
    Eigen::Matrix<double, 6, 6> matrix_inverse(const Eigen::Matrix<double, 6, 6> &matrix);

    Eigen::Matrix3d inertia_matrix(const std::vector<PointMass> &points);
    Eigen::Matrix<double, 6, 6> mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix);
    Eigen::Matrix<double, 6, 6> coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu);
    Eigen::Matrix3d rotation_matrix_eb(const Eigen::Vector3d &attitude);
    Eigen::Matrix3d transformation_matrix(const Eigen::Vector3d &attitude);
    Eigen::Matrix<double, 6, 6> J_Theta(const Eigen::Vector<double, 6> &eta);
};
