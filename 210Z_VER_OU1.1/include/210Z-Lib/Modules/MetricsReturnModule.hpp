#include "iostream"

namespace Eclipse{
    class Metrics{
        private:
            bool init;
        public:
            void output_sensor_data();
            void output_auton_selector();
            void output_game_data();
            void output_misc_data();
            void display_data();
    };
}
