#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <sstream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rh_plus_interface/msg/servo_read_data.hpp"

#include "servo/servo_command.hpp"
#include "global_variable.hpp"
#include "servo/servo_control.hpp"



ServoControlNode::ServoControlNode(const rclcpp::NodeOptions & options)
: Node("motor_control", options)
{
  using namespace std::placeholders;

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  motor_data_pub_ = this->create_publisher<ReadData>("servo_info", QOS_RKL10V);

  // IK Topic received
  ik_result_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",QOS_RKL10V,std::bind(&ServoControlNode::ik_angle_callback, this, std::placeholders::_1));

    this->timer_=
      this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ServoControlNode::servo_info_pub, this));

}

ServoControlNode::~ServoControlNode()
{
}

void ServoControlNode::ik_angle_callback(const sensor_msgs::msg::JointState & msg)
{
  for (int i=0; i < SERVO_NUM; i++){
    angles[i] = round(500 + msg.position[i] * 180 / M_PI * 1000 / 240);
    write_angle(i+1,angles[i]);
  }

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

void ServoControlNode::servo_info_pub()
{
  auto message = ReadData();

  int * arr_temp = read_temp(SERVO_NUM);
  unsigned short int * arr_angle = read_angle(SERVO_NUM);
  for (int i=0; i<SERVO_NUM; i++){
    message.angle_data.push_back(arr_angle[i]);
    message.temp_data.push_back(arr_temp[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Servo Information transmit!! temp: %d  %d, angle: %d  %d \n",message.temp_data[0], message.temp_data[1], message.angle_data[0], message.angle_data[1]);
  motor_data_pub_->publish(message);
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
