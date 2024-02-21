#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>
#include <math.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"
#include "servo/servo_control.hpp"



ServoControlNode::ServoControlNode(const rclcpp::NodeOptions & options)
: Node("motor_control", options)
{
  using namespace std::placeholders;

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // IK Topic received
  ik_result_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",QOS_RKL10V,std::bind(&ServoControlNode::ik_angle_callback, this, std::placeholders::_1));
}

ServoControlNode::~ServoControlNode()
{
}

void ServoControlNode::ik_angle_callback(const sensor_msgs::msg::JointState & msg)
{
  for (int i=0; i < SERVO_NUM; i++){
    if (i == (SERVO_NUM-1))
      angles[i] = round(500 + length2angle(msg.position[i]) * 180 / M_PI * 1000 / 240);
    else
      angles[i] = round(500 + msg.position[i] * 180 / M_PI * 1000 / 240);
    write_angle(i+1,angles[i]);
  }

  // 터미널 확인용
  int angles_degree[SERVO_NUM] = {0,};
  for (int i=0; i <SERVO_NUM; i++){
    angles_degree[i] = round(msg.position[i] * 180 / M_PI);
  }
  std::stringstream ss;
  ss << "Transmit the angle: ";

  for (int i=0; i < SERVO_NUM; i++){
    ss << angles_degree[i] << " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s",ss.str().c_str());
}

double length2angle(double length)
{
  // 상수 설정
  const double a = 0.026000;
  const double b = 0.031000;
  const double c = 0.010048;
  const double d = 0.001552;

  // 변수 y 정의
  double y = length + std::sqrt(b * b + (a - c) * (a - c)) - d;

  // y의 연산을 통한 angle 계산
  double angle = std::asin((y * y + a * a + c * c - b * b) / (2 * a * std::sqrt(y * y + c * c))) -
                std::asin(c / std::sqrt(y * y + c * c));
  // angle 반환
  return angle;
}

// Node actually executes
int main(int argc, char * argv[])
{
  // ROS 2 initializes
  rclcpp::init(argc, argv);
  // starts processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<ServoControlNode>());
  rclcpp::shutdown();
  return 0;
}
