#ifndef _FEEDBACKS_H
#define _FEEDBACKS_H

#define GYRO_LONG_INTEGRAL_IGNORE_LEVEL 0

#define MAX_DIFFERENTIAL_SPEED 3000
#define MAX_SPEED 6000

void run_feedbacks(int sens_status);
void move_arc_manual(int comm, int ang);
void compass_fsm(int cmd);
void sync_to_compass();
void host_alive();
void rotate_rel(int angle);
void rotate_abs(int angle);
void straight_rel(int fwd /*in mm*/);
int correcting_angle();
int correcting_straight();
int correcting_either();
int robot_moving();
void take_control();
void reset_movement();

#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047


typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // with lidars: mm. With cur_pos: 1/10 mm.
	int32_t y;
} pos_t;

#define COPY_POS(to, from) { (to).ang = (from).ang; (to).x = (from).x; (to).y = (from).y; }
#define COPY_POS_PER10(to, from) { (to).ang = (from).ang; (to).x = (from).x/10; (to).y = (from).y/10; }

extern pos_t cur_pos;

void zero_angle();
void zero_coords();

void allow_angular(int yes);
void allow_straight(int yes);
void auto_disallow(int yes);

void correct_location_without_moving(pos_t corr);

#endif
