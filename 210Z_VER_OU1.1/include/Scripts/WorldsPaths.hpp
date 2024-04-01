/**
 * @file WorldsPaths.hpp
 * @author Zechariah Wang
 * @brief Odometry logic for global position tracking within robot using cartesian coordinates
 * @version 0.1
 * @date 2024-03-28
 * 
 */

#include "iostream"

namespace Eclipse {
    class Paths {
        public:
            void local_close_side();
            void local_six_ball();

            void global_close_side();
            void global_six_ball();

            void rush_six_ball();
            void rush_disruption_close_side();

    };
}