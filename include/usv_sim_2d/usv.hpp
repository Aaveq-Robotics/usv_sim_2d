#pragma once

#include <chrono>
#include <time.h>
#include <Eigen/Dense>

#define SERVO_MIN 1100
#define SERVO_TRIM 1500
#define SERVO_MAX 1900

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

    std::vector<PointMass> point_list_body_;
    std::vector<Eigen::Vector3d> point_list_earth_;
    double mass_;

    USV();
    ~USV() {}

    Eigen::Vector<double, 6> compute_forces(const std::array<uint16_t, 16> &servo_out);
    bool rigid_body_dynamics(const Eigen::Vector<double, 6> &tau);

protected:
    // Member variables
    Eigen::Vector<double, 6> nu_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector<double, 6> eta_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::Matrix3d inertia_matrix_;
    Eigen::Matrix<double, 6, 6> mass_matrix_;

    // Member functions
    double get_time();
    double update_timestamp();
    void set_initial_condition(const Eigen::Vector<double, 6> &initial_condition);
    double sum_mass(const std::vector<PointMass> &points);
    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);

    std::vector<PointMass> recompute_relative_to_origin(const std::vector<PointMass> &points);
    Eigen::Matrix3d inertia_matrix(const std::vector<PointMass> &points);
    Eigen::Matrix<double, 6, 6> mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix);
    Eigen::Matrix<double, 6, 6> coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu);
    Eigen::Matrix3d rotation_matrix_eb(const Eigen::Vector3d &attitude);
    Eigen::Matrix3d transformation_matrix(const Eigen::Vector3d &attitude);
    Eigen::Matrix<double, 6, 6> J_Theta(const Eigen::Vector<double, 6> &eta);
};
