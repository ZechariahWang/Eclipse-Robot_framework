#include "iostream"

extern uint8_t auton_finalized; // Final auton choice
extern uint8_t selected_auton; // Auton choice
extern uint8_t global_auton; // Auton choice

namespace Eclipse{
    class Selector{
        private:
            bool init;
        public:
            void receive_selector_input(u_int32_t time);
            void select_current_auton();
            void reset_all_primary_sensors();
    };
}