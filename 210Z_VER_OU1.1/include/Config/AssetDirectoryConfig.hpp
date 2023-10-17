#include "iostream"
#include "vector"

#pragma once
#include "main.h"

class AssetConfig{
    public:
        AssetConfig();
        AssetConfig(std::vector<int> left_chassis_ports,
                    std::vector<int> right_chassis_ports);

        int8_t front_left_port;
        int8_t middle_left_port;
        int8_t rear_left_port;

        int8_t front_right_port;
        int8_t middle_right_port;
        int8_t rear_right_port;

        bool left_reversed;
        bool right_reversed;

};

extern AssetConfig config;
extern std::vector<pros::Motor> chassis_left_motors;
extern std::vector<pros::Motor> chassis_right_motors;