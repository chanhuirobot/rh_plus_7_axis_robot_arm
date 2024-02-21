#include "rclcpp/rclcpp.hpp"
#include "rh_plus_interface/srv/servoinfos.hpp"

#include <stdint.h>

class ServoInfoClient : public rclcpp::Node
{
    public:
        ServoInfoClient() : Node("servo_info_client")
        {
            client_ = create_client<rh_plus_interface::srv::Servoinfos>("/servo_infos");

            while (rclcpp::ok())
            {
                uint8_t input;
                std::cout << "Enter a number (1: Temp data, 2: Angle data, 0: exit): ";
                std::cin >> input;

                if (input == 0 || (input != 1 && input != 2))
                    break;

                auto request = std::make_shared<rh_plus_interface::srv::Servoinfos::Request>();
                request->input = input;

                auto result = client_->async_send_request(request);

                try {
                    auto response = result.get();
                    RCLCPP_INFO(get_logger(), "Received response: ");
                    if (input == 1)
                        for (size_t i = 0; i < response->servo_temps.size(); ++i)
                            RCLCPP_INFO(this->get_logger(), "Servo Temp[%zu]: %u\\n", i, response->servo_temps[i]);
                    else if (input == 2)
                        for (size_t i = 0; i < response->servo_angles.size(); ++i)
                            RCLCPP_INFO(this->get_logger(), "Servo Angle[%zu]: %d\\n", i, response->servo_angles[i]);
                }
                catch(const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service Call failed");
                }
            }
        }
    private:
        rclcpp::Client<rh_plus_interface::srv::Servoinfos>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<ServoInfoClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
