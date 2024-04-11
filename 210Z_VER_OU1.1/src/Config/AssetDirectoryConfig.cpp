/**
 * @file AssetDirectoryConfig.cpp
 * @author Zechariah Wang
 * @brief Drivetrain config driver
 * @version 0.1
 * @date 2024-04-05
 */

#include "AssetDirectoryConfig.hpp"
#include "Globals.hpp"

using namespace Eclipse;

std::vector<pros::Motor> chassis_left_motors;
std::vector<pros::Motor> chassis_right_motors;

AssetConfig::AssetConfig(std::vector<int> left_chassis_ports,
                         std::vector<int> right_chassis_ports) {
                            
    config.front_left_port = left_chassis_ports.at(0);
    config.middle_left_port = left_chassis_ports.at(1);
    config.rear_left_port = left_chassis_ports.at(2);

    config.front_right_port = right_chassis_ports.at(0);
    config.middle_right_port = right_chassis_ports.at(1);
    config.rear_right_port = right_chassis_ports.at(2);

    for (auto i : left_chassis_ports) {
        pros::Motor temp(abs(i), utility::is_reversed(i));
        chassis_left_motors.push_back(temp);
    }
    for (auto i : right_chassis_ports) {
        pros::Motor temp(abs(i), utility::is_reversed(i));
       chassis_right_motors.push_back(temp);
    }

    config.left_reversed = true;
    config.right_reversed = false;

}

