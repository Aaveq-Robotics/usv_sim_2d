#include <chrono>
#include <time.h>
#include <Eigen/Dense>

class USV
{
public:
    struct VechicleState
    {
        double timestamp = 0;

        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
        Eigen::Vector3d position;
        Eigen::Vector3d attitude;
        Eigen::Vector3d velocity;
    } state, state_old;

    USV();
    ~USV() {}

    bool update(std::array<uint16_t, 16> servo_out);

private:
    uint64_t micros();

    double _interp1D(const double &x, const double &x0, const double &x1, const double &y0, const double &y1);
};
