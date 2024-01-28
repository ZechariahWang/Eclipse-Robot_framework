#include "iostream"

void power_intake();
void prime_catapult();
void activate_cata();
void raw_cata();
void extend_wings();
void extend_left_wing();
void extend_right_wing();
void extend_blocker();
void extend_front_wings();
void extend_climber();
void stop_cata_with_sensor();
void extend_odom_piston();
void controlFlywheel(int targetVelocity);
void init_sequence();
void auton_sequence();

extern bool arm_down;
extern bool down_flywheel_enabled;
extern bool cata_toggled;