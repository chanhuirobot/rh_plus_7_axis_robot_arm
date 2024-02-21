// server
#include "rclcpp/rclcpp.hpp"
#include "rh_plus_interface/srv/servoinfos.hpp"
#include "servo/servo_command.hpp"
#include "global_variable.hpp"

class ServoInfoServer : public rclcpp::Node
{
public:
  ServoInfoServer() : Node("servo_info_server")
  {
    // 서비스 서버 생성
    server_ = create_service<rh_plus_interface::srv::Servoinfos>(
      "/servo_infos",
      std::bind(&ServoInfoServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_service_request(const std::shared_ptr<rh_plus_interface::srv::Servoinfos::Request> request,
                              std::shared_ptr<rh_plus_interface::srv::Servoinfos::Response> response)
  {
    // UART 통신 후에 값을 넣기
    if (request->input == 1){
      RCLCPP_INFO(get_logger(), "Received request Tempeature\\n");
      int * arr_temp = read_temp(SERVO_NUM);

      response->servo_temps.clear();
      response->servo_temps.insert(response->servo_temps.end(), arr_temp, arr_temp + SERVO_NUM);
    }
    else{
      RCLCPP_INFO(get_logger(), "Received request Angle\\n");
      unsigned short int * arr_angle = read_angle(SERVO_NUM);

      response->servo_angles.clear();
      response->servo_angles.insert(response->servo_angles.end(), arr_angle, arr_angle + SERVO_NUM);
     }

  }

  rclcpp::Service<rh_plus_interface::srv::Servoinfos>::SharedPtr server_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<ServoInfoServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
