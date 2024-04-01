/**
 * @file WorldsPath.cpp
 * @author Zechariah Wang
 * @brief All paths for Worlds 2024
 * @version 0.1
 * @date 2024-03-28
 */

#include "main.h"

using namespace Eclipse;

void Eclipse::Paths::local_close_side() {
	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(48, 110, 3, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(180, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(-48, 110, 3, true);

    cur_c.set_c_constants(3, 0, 30);
    cur_c.set_curve_pid(90, 90, 0.3, 3, false);
}

void Eclipse::Paths::local_six_ball() {

}

void Eclipse::Paths::global_close_side() {

}
void Eclipse::Paths::global_six_ball() {

}

void Eclipse::Paths::rush_six_ball() {

}
void Eclipse::Paths::rush_disruption_close_side() {

}