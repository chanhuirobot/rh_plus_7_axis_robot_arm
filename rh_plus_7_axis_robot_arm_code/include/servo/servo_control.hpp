#ifndef SERVO_CONTROL_HPP_
#define SERVO_CONTROL_HPP_

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"

double length2angle(double length);

class ServoControlNode : public rclcpp::Node
{
  public:
    explicit ServoControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~ServoControlNode();

    int angles[SERVO_NUM] = {0,};
  private:

    void ik_angle_callback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ik_result_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};


#endif
