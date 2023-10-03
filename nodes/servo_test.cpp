#include <array>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>

#include "aaveq_ros_interfaces/msg/control_output.hpp"

class ServoTest : public rclcpp::Node
{
public:
    ServoTest() : Node("servor_test")
    {
        /***** Parameters *****/
        freq_ = this->declare_parameter<double>("freq", 60.0);

        /***** Publishers *****/
        publisher_ = this->create_publisher<aaveq_ros_interfaces::msg::ControlOutput>("control_output", 1);

        /***** Timers *****/
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_), std::bind(&ServoTest::callback_timer, this));
    }

private:
    /***** Variables *****/
    // Node parameters
    double freq_;

    // Node variables
    rclcpp::Publisher<aaveq_ros_interfaces::msg::ControlOutput>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    std::array<uint16_t, 16> servo_out_;

    /***** Functions *****/
    double get_time()
    {
        uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        return (double)us / 1000000.0; // Convert to seconds
    }

    /***** Callbacks *****/
    void callback_timer()
    {
        static const double amplitude = 200.0;
        static const double frequency = 1.0 / 10.0;
        static const double phase = 0.0;
        static const double offset = 1700.0;
        double time = get_time();

        double chn_0 = amplitude * sin(2 * M_PI * frequency * time + phase) + offset;
        double chn_2 = amplitude * sin(2 * M_PI * frequency * time + (phase + M_PI)) + offset;

        servo_out_ = {(uint16_t)chn_0, 0, (uint16_t)chn_2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        aaveq_ros_interfaces::msg::ControlOutput msg_servo;
        std::copy(servo_out_.begin(), servo_out_.end(),
                  std::back_inserter(msg_servo.servo_pwm));

        publisher_->publish(msg_servo);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoTest>());
    rclcpp::shutdown();
    return 0;
}
