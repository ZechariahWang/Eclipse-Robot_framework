#include "AssetDirectoryConfig.hpp"

using namespace Eclipse;

std::vector<pros::Motor> chassis_left_motors;
std::vector<pros::Motor> chassis_right_motors;

AssetConfig::AssetConfig(std::vector<int> left_chassis_ports,
                         std::vector<int> right_chassis_ports,
                        bool left_reversed,
                        bool right_reversed) {
                            
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

    config.left_reversed = left_reversed;
    config.right_reversed = right_reversed;
    
}

