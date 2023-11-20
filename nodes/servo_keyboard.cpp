#include <array>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <SFML/Graphics.hpp>

#include "aaveq_ros_interfaces/msg/control_output.hpp"

class ServoKeyboard : public rclcpp::Node
{
public:
    ServoKeyboard() : Node("servor_keyboard")
    {
        /***** Parameters *****/
        freq_ = this->declare_parameter<double>("freq", 60.0);

        /***** Publishers *****/
        publisher_ = this->create_publisher<aaveq_ros_interfaces::msg::ControlOutput>("control_output", 1);

        /***** Timers *****/
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / freq_), std::bind(&ServoKeyboard::callback_timer, this));
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
        uint16_t cw = 1500;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            cw = 1900;

        uint16_t ccw = 1500;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            ccw = 1900;

        servo_out_ = {cw, 0, ccw, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        aaveq_ros_interfaces::msg::ControlOutput msg_servo;
        std::copy(servo_out_.begin(), servo_out_.end(),
                  std::back_inserter(msg_servo.servo_pwm));

        publisher_->publish(msg_servo);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoKeyboard>());
    rclcpp::shutdown();
    return 0;
}
