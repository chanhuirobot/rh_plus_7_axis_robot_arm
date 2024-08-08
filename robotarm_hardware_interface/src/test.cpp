// 순수 조인트 테스트... 절대로 로봇에 물려서 작동하면 안됨!!!
// 모터 7개 단독으로 있을 때만 사용

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "robotarm_hardware_interface/robotarm_serial.hpp"
#include "robotarm_hardware_interface/robotarm_drvr.hpp"

const int NUM_JOINTS = 7;

const std::string SERIAL_DEV = "/dev/ttyUSB0";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int i;
    std::unique_ptr<robotarm::robotarm_drvr> drvr_ = std::make_unique<robotarm::robotarm_serial>();

    if (!drvr_->open(SERIAL_DEV))
    {
        std::cerr << "Failed to open driver";
        return false;
    }

    // 모두 센터(500) 맞추기
    for (i = 1; i < NUM_JOINTS + 1; i = i + 1)
    {
        drvr_->setJointPosition(i, 500, 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    for (i = 1; i < NUM_JOINTS + 1; i = i + 1)
    {
        // i번 모터
        if (!drvr_->setJointPosition(i, 400, 500))
        {
            std::cerr << "Failed to set servo position" << std::endl;
        }
        else
        {
            std::cerr << "Set joint position motor " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!drvr_->setJointPosition(i, 600, 500))
        {
            std::cerr << "Failed to set servo position" << std::endl;
        }
        else
        {
            std::cerr << "Set joint position motor " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!drvr_->setJointPosition(i, 500, 500))
        {
            std::cerr << "Failed to set servo position" << std::endl;
        }
        else
        {
            std::cerr << "Set joint position motor " << i << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cerr << "Try moving it by hand - should be free to move" << std::endl;
    drvr_->setManualModeAll(true, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    std::cerr << "Try moving it by hand - should be holding position" << std::endl;
    drvr_->setManualModeAll(false, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    std::cerr << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
