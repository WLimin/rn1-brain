/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

#ifndef _FEEDBACKS_H
#define _FEEDBACKS_H

#include "main.h"

#define GYRO_LONG_INTEGRAL_IGNORE_LEVEL 0

#define MAX_DIFFERENTIAL_SPEED 3000
#define MAX_SPEED 6000
#ifdef   __cplusplus
extern "C" {
#endif

void run_feedbacks(uint32_t sens_status);
void move_arc_manual(int comm, int ang);
void compass_fsm(int cmd);
void sync_to_compass(void);
void host_alive(void);
void host_alive_long(void);
void host_dead(void);
void rotate_rel(int angle);
void rotate_abs(int angle);
void straight_rel(int fwd /*in mm*/);
int correcting_angle(void);
int angle_almost_corrected(void);
int get_ang_err(void);
int correcting_straight(void);
int correcting_either(void);
int robot_moving(void);
void take_control(void);
void reset_movement(void);
void speed_limit(int new_status);

void set_top_speed_ang(int speed);
void set_top_speed_ang_max(int speed); // Can only lower the existing limits

void set_top_speed_fwd(int speed);
void set_top_speed_fwd_max(int speed); // Can only lower the existing limits

// Sets both angular and forward:
void set_top_speed(int speed);
void set_top_speed_max(int speed); // Can only lower the existing limits

void reset_speed_limits(void);

void enable_collision_detection(void);

void change_angle_to_cur(void);


int get_fwd(void);

#define ANG_180_DEG (uint32_t)2147483648UL
#define ANG_90_DEG  (uint32_t)1073741824
#define ANG_2_5_DEG   (uint32_t)29826162
#define ANG_1_DEG     (uint32_t)11930465
#define ANG_0_5_DEG    (uint32_t)5965232
#define ANG_0_25_DEG   (uint32_t)2982616
#define ANG_0_125_DEG  (uint32_t)1491308
#define ANG_0_1_DEG    (uint32_t)1193047
#define ANG_0_05_DEG    (uint32_t)596523
#define ANG_0_01_DEG    (uint32_t)119305
#define ANG_0_001_DEG    (uint32_t)11930

#define ANG_1PER16_DEG  (uint32_t)745654  // cumulated full circle rounding error: 0.000006%

#define RAD2nDEG  (ANG_180_DEG / M_PI)		//convert rad to 10^-9 degree
#define ANG_DEG(a) ((a) * ANG_1_DEG)

#define INT2mm_Unit		(uint32_t)0x10000 //1 mm is 2^16 unit.

#define mm2Int(v)	((v) * INT2mm_Unit)
#define Int2mm(v)	((v) / INT2mm_Unit)

#define Ang2Int32(v)	((v) * (uint32_t)0x10000)
#define Int32toAng(v)	((v) / (uint32_t)0x10000)

#define XCEL_X_NEG_WARN ((int)(-20000)*256)
#define XCEL_X_POS_WARN ((int)(20000)*256)
#define XCEL_Y_NEG_WARN ((int)(-16000)*256)
#define XCEL_Y_POS_WARN ((int)(16000)*256)

#define XCEL_X_NEG_COLL ((int)(-27000)*256)
#define XCEL_X_POS_COLL ((int)(27000)*256)
#define XCEL_Y_NEG_COLL ((int)(-22000)*256)
#define XCEL_Y_POS_COLL ((int)(22000)*256)


typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.8190317153931 ndeg
	int32_t x;   // mm
	int32_t y;   // mm
} pos_t;

#define COPY_POS(to, from) { (to).ang = (from).ang; (to).x = (from).x; (to).y = (from).y; }

extern volatile pos_t cur_pos;

void zero_angle(void);
void zero_coords(void);

void allow_angular(bool yes);
void allow_straight(bool yes);
void auto_disallow(bool yes);

void correct_location_without_moving(pos_t corr);
void correct_location_without_moving_external(pos_t corr);
void set_location_without_moving_external(pos_t new_pos);

void change_angle_abs(int angle);
void change_angle_rel(int angle);
void change_straight_rel(int fwd /*in mm*/);

void dbg_teleportation_bug(int id);

typedef struct __attribute__((packed))
{
	int32_t id;
	int32_t prev_id;
	int32_t prev2_id;
	int32_t prev3_id;
	int32_t prev4_id;
	int64_t prev_x;
	int64_t prev_y;
	int64_t cur_x;
	int64_t cur_y;
} dbg_teleportation_bug_data_t;

typedef struct __attribute__((packed))
{
	int32_t wd0;
	int32_t wd1;
	int32_t movement;
	int32_t x_idx;
	int32_t y_idx;
	int64_t dx;
	int64_t dy;
	int64_t x_before;
	int64_t y_before;
	int64_t x_after;
	int64_t y_after;

} dbg_teleportation_extra_t;
#ifdef   __cplusplus
}
#endif

extern volatile dbg_teleportation_extra_t dbg_teleportation_extra, dbg_teleportation_extra_to_send;


extern volatile int dbg_teleportation_bug_report;
extern volatile dbg_teleportation_bug_data_t dbg_teleportation_bug_data;

// Temporarily here, relayed to motor controllers, for adjusting PID loops.
extern volatile uint8_t mc_pid_imax;
extern volatile uint8_t mc_pid_feedfwd;
extern volatile uint8_t mc_pid_p;
extern volatile uint8_t mc_pid_i;
extern volatile uint8_t mc_pid_d;


#endif
