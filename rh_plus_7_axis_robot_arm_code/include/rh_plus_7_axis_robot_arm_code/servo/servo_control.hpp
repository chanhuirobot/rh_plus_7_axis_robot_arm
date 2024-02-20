#ifndef SERVO_CONTROL_HPP_
#define SERVO_CONTROL_HPP_

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"


class ServoControlNode : public rclcpp::Node
{
  public:
    using ReadData = rh_plus_interface::msg::ServoReadData;

    explicit ServoControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ServoControlNode();

    int angles[SERVO_NUM] = {0,};

  private:

    void ik_angle_callback(const sensor_msgs::msg::JointState & msg);
    void servo_info_pub();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ik_result_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ReadData>::SharedPtr motor_data_pub_;

};


#endif
