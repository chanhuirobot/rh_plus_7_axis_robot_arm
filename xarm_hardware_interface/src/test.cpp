
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "xarm_hardware_interface/xarm_serial.hpp"
#include "xarm_hardware_interface/xarm_drvr.hpp"


const int NUM_JOINTS = 7; // 조인트 개수

const std::string SERIAL_DEV = "/dev/servo_driver";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int i;
    std::unique_ptr<xarm::xarm_drvr> drvr_ = std::make_unique<xarm::xarm_serial>(); // 객체 생성
    uint16_t pos;

    if (!drvr_->open(SERIAL_DEV)) {
        std::cerr << "Failed to open driver";
        return false;
    }

	if (!drvr_->setJointPosition(1, 200, 2000)) { // 200각도에 2000ms 동안
        std::cerr << "Failed to set servo position" << std::endl;
    } else {
        std::cerr << "Set joint position" << std::endl;
    }

    for (i = 0; i < 5*2; i++) { // 10번에 걸쳐서 서보 각도 읽기
        std::this_thread::sleep_for(std::chrono::milliseconds(200) ); // 200ms 딜레이

        if (drvr_->getJointPosition(1, pos)) {
            std::cout << "servo " << i << ", pos: " << pos << std::endl;
        } else {
            std::cerr << "Failed to read servo position" << std::endl;
        }
    }

    if (!drvr_->setJointPosition(1, 800, 2000)) {
        std::cerr << "Failed to set servo position" << std::endl;
    } else {
        std::cerr << "Set joint position" << std::endl;
    }

    for (i = 0; i < 5*2; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200) );

        if (drvr_->getJointPosition(1, pos)) {
            std::cout << "servo " << i << ", pos: " << pos << std::endl;
        } else {
            std::cerr << "Failed to read servo position" << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000) );

    if (drvr_->getJointPosition(1, pos)) {
        std::cout << "servo " << i << ", pos: " << pos << std::endl;
    } else {
        std::cerr << "Failed to read servo position" << std::endl;
    }

    std::cerr << "Try moving it by hand - should be free to move" << std::endl; // 토크 끄기
	drvr_->setManualModeAll(true, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000) );

    std::cerr << "Try moving it by hand - should be holding position" << std::endl; // 토크 켜기
	drvr_->setManualModeAll(false, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000) );

    std::cerr << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
