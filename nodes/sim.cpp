#include <array>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "aaveq_ros_interfaces/msg/control_output.hpp"
#include "aaveq_ros_interfaces/msg/sim_state.hpp"
#include "usv_sim_2d/visualisation.hpp"
#include "usv_sim_2d/usv.hpp"

class Sim : public rclcpp::Node
{
public:
    Sim() : Node("sim")
    {
        /***** Parameters *****/
        sim_freq_ = this->declare_parameter<double>("sim_freq", 60.0);
        vessel_config_path_ = this->declare_parameter<std::string>("vessel_config", "");

        /***** Subscription *****/
        subscriber_control_output_ = this->create_subscription<aaveq_ros_interfaces::msg::ControlOutput>("control_output", 1, std::bind(&Sim::callback_control_output, this, std::placeholders::_1));

        /***** Publishers *****/
        publisher_sim_state_ = this->create_publisher<aaveq_ros_interfaces::msg::SimState>("sim_state", 1);

        /***** Timers *****/
        timer_sim_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / sim_freq_), std::bind(&Sim::callback_timer_sim, this));
        timer_window_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 60), std::bind(&Sim::callback_timer_window, this));

        /***** Initiate Variables *****/
        usv_.load_vessel_config(vessel_config_path_);
    }

private:
    /***** Variables *****/
    // Node parameters
    double sim_freq_;
    std::string vessel_config_path_;

    // Node variables
    rclcpp::Subscription<aaveq_ros_interfaces::msg::ControlOutput>::SharedPtr subscriber_control_output_;
    rclcpp::Publisher<aaveq_ros_interfaces::msg::SimState>::SharedPtr publisher_sim_state_;
    rclcpp::TimerBase::SharedPtr timer_sim_;
    rclcpp::TimerBase::SharedPtr timer_window_;

    // Variables
    Visualisation window_;

    USV usv_;
    std::array<uint16_t, 16> servo_out_;

    /***** Callbacks *****/
    void callback_control_output(const aaveq_ros_interfaces::msg::ControlOutput::SharedPtr msg_servo_out)
    {
        std::copy(msg_servo_out->servo_pwm.begin(), msg_servo_out->servo_pwm.end(),
                  servo_out_.begin());
    }

    void callback_timer_sim()
    {
        // Calculate USV physics
        Eigen::Vector<double, 6> tau = usv_.compute_forces(servo_out_);

        if (!usv_.rigid_body_dynamics(tau))
        {
            RCLCPP_WARN_STREAM(get_logger(), "Physics update has caused an exit");
            return;
        };

        static aaveq_ros_interfaces::msg::SimState msg_state;

        msg_state.timestamp = usv_.state.timestamp;

        msg_state.gyro.x = usv_.state.gyro.x(); // Roll
        msg_state.gyro.y = usv_.state.gyro.y(); // Pitch
        msg_state.gyro.z = usv_.state.gyro.z(); // Yaw

        msg_state.accel.x = usv_.state.accel.x();
        msg_state.accel.y = usv_.state.accel.y();
        msg_state.accel.z = usv_.state.accel.z();

        msg_state.position.x = usv_.state.position.x();
        msg_state.position.y = usv_.state.position.y();
        msg_state.position.z = usv_.state.position.z();

        msg_state.attitude.x = usv_.state.attitude.x(); // Roll
        msg_state.attitude.y = usv_.state.attitude.y(); // Pitch
        msg_state.attitude.z = usv_.state.attitude.z(); // Yaw

        msg_state.velocity.x = usv_.state.velocity.x();
        msg_state.velocity.y = usv_.state.velocity.y();
        msg_state.velocity.z = usv_.state.velocity.z();

        publisher_sim_state_->publish(msg_state);
    }

    void callback_timer_window()
    {
        window_.update(usv_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim>());
    rclcpp::shutdown();
    return 0;
}
