#include <array>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "aaveq_ros_interfaces/msg/control_output.hpp"
#include "aaveq_ros_interfaces/msg/sim_state.hpp"
#include "usv_sim_2d/usv.hpp"

class Sim : public rclcpp::Node
{
public:
    Sim() : Node("sim")
    {
        /***** Parameters *****/
        sim_freq_ = this->declare_parameter<double>("sim_freq", 1.0);

        /***** Subscription *****/
        subscriber_control_output_ = this->create_subscription<aaveq_ros_interfaces::msg::ControlOutput>("control_output", 1, std::bind(&Sim::callback_control_output, this, std::placeholders::_1));

        /***** Publishers *****/
        publisher_sim_state_ = this->create_publisher<aaveq_ros_interfaces::msg::SimState>("sim_state", 1);

        /***** Timers *****/
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / sim_freq_), std::bind(&Sim::callback_timer, this));

        /***** Variables *****/
        servo_out_.fill(1500);
    }

private:
    /***** Variables *****/
    // Node parameters
    double sim_freq_;

    // Node variables
    rclcpp::Subscription<aaveq_ros_interfaces::msg::ControlOutput>::SharedPtr subscriber_control_output_;
    rclcpp::Publisher<aaveq_ros_interfaces::msg::SimState>::SharedPtr publisher_sim_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    USV usv_;
    std::array<uint16_t, 16> servo_out_;

    /***** Callbacks *****/
    void callback_control_output(const aaveq_ros_interfaces::msg::ControlOutput::SharedPtr msg_servo_out)
    {
        std::copy(msg_servo_out->servo_pwm.begin(), msg_servo_out->servo_pwm.end(),
                  servo_out_.begin());
    }

    void callback_timer()
    {
        // calc rover physics
        if (!usv_.update(servo_out_))
        {
            // something went wrong with the physics
            RCLCPP_WARN_STREAM(get_logger(), "Physics update has caused an exit");
            return;
        };

        static aaveq_ros_interfaces::msg::SimState msg_state;

        msg_state.timestamp = usv_.state.timestamp;

        msg_state.gyro.x = usv_.state.gyro[0];
        msg_state.gyro.y = usv_.state.gyro[1];
        msg_state.gyro.z = usv_.state.gyro[2];

        msg_state.accel.x = usv_.state.accel[0];
        msg_state.accel.y = usv_.state.accel[1];
        msg_state.accel.z = usv_.state.accel[2];

        msg_state.position.x = usv_.state.position[0];
        msg_state.position.y = usv_.state.position[1];
        msg_state.position.z = usv_.state.position[2];

        msg_state.attitude.x = usv_.state.attitude[0];
        msg_state.attitude.y = usv_.state.attitude[1];
        msg_state.attitude.z = usv_.state.attitude[2];

        msg_state.velocity.x = usv_.state.velocity[0];
        msg_state.velocity.y = usv_.state.velocity[1];
        msg_state.velocity.z = usv_.state.velocity[2];

        publisher_sim_state_->publish(msg_state);

        // Debugging
        RCLCPP_INFO_STREAM(get_logger(), " ------ " << msg_state.timestamp << " ------ ");
        for (auto i : servo_out_)
            RCLCPP_INFO_STREAM(get_logger(), i << " ");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim>());
    rclcpp::shutdown();
    return 0;
}
