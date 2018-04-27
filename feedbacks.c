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


	Mechanical feedback module.

	Integrates the sensors. Keeps track of position & angle, controls the motors.
*/


#include <inttypes.h>
#include <math.h>

#include "ext_include/stm32f2xx.h"
#include "feedbacks.h"
#include "gyro_xcel_compass.h"
#include "motcons.h"
#include "sin_lut.h"
#include "navig.h"
#include "main.h"

#define STEP_FEEDFORWARD_ANG 22000  // was 22 000 for a long time
#define STEP_FEEDFORWARD_FWD 100000  // was 100 000 for a long time
#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

// Which motor controllers are in use on the PCB
#ifdef RN1P4
	#define A_MC_IDX 2
	#define B_MC_IDX 3
#endif
#ifdef RN1P6
	#define A_MC_IDX 3
	#define B_MC_IDX 0
#endif
#ifdef RN1P7
	#define A_MC_IDX 3
	#define B_MC_IDX 2
#endif
#ifdef PULU1
	#define A_MC_IDX 3
	#define B_MC_IDX 0
#endif

#ifdef PROD1
	#define A_MC_IDX 1
	#define B_MC_IDX 0
#endif

extern volatile int dbg[10];

// Temporarily here, relayed to motor controllers, for adjusting PID loops.
volatile uint8_t mc_pid_imax = 30;
volatile uint8_t mc_pid_feedfwd = 30;
volatile uint8_t mc_pid_p = 80;
volatile uint8_t mc_pid_i = 80;
volatile uint8_t mc_pid_d = 50;

static bool reset_wheel_slip_det;

uint8_t feedback_stop_flags;
int feedback_stop_param1, feedback_stop_param2;
int feedback_stop_param1_store, feedback_stop_param2_store;

int64_t gyro_long_integrals[3];
int64_t gyro_short_integrals[3];

int64_t xcel_long_integrals[3];
int64_t xcel_short_integrals[3];

int64_t xcel_dc_corrs[3];
int64_t gyro_dc_corrs[3];

// cur x,y are being integrated at huge (1mm/(2^32)) resolution; the result is copied in millimeters to cur_pos.
static volatile int64_t cur_x;
static volatile int64_t cur_y;
volatile pos_t cur_pos;

int gyro_timing_issues;
int xcel_timing_issues;

int cur_compass_angle = 0;
int aim_angle = 0;
int ang_top_speed = 100000;
int ang_p = 900;

int aim_fwd;
int final_fwd_accel = 400;
int fwd_accel = 350;
int fwd_top_speed = 600000;
int fwd_p = 1600;

volatile int manual_control;
volatile int manual_common_speed;
volatile int manual_ang_speed;

int robot_nonmoving_cnt = 0;
int robot_nonmoving = 0;	//1 = robot is static, 0 = robot is moving.
//set robot moved flag.
static void set_robot_moving(void) {
	robot_nonmoving_cnt = 0;
	robot_nonmoving = 0;
}

int robot_moving(void) {
	return !robot_nonmoving;
}

static int host_alive_watchdog;
void host_alive(void) {
	host_alive_watchdog = 5000;
}

void host_alive_long(void) {
	host_alive_watchdog = 10000;
}

void host_dead(void) {
	host_alive_watchdog = 0;
}

void zero_gyro_short_integrals(void) {
	gyro_short_integrals[0] = 0;
	gyro_short_integrals[1] = 0;
	gyro_short_integrals[2] = 0;
}

void zero_xcel_short_integrals(void) {
	xcel_short_integrals[0] = 0;
	xcel_short_integrals[1] = 0;
	xcel_short_integrals[2] = 0;
}

void zero_xcel_long_integrals(void) {
	xcel_long_integrals[0] = 0;
	xcel_long_integrals[1] = 0;
	xcel_long_integrals[2] = 0;
}

static int fwd_speed_limit;
static int ang_speed_limit;
static int speed_limit_lowered;

static bool do_correct_angle = false;
static bool do_correct_fwd   = false;

void take_control(void) {
	manual_control = 0;
}

int speed_limit_status(void) {
	return speed_limit_lowered;
}

#define MIN_SPEED_ANG 60000  // has been 70000 very long
#define MIN_SPEED_FWD 60000 // has been 110000 very long

#define FWD_SPEED_MUL 10000 // from 12000 -> 8000 due to slow lidar -> back to 10000

#if defined(RN1P4) || defined(RN1P6) || defined(RN1P7) || defined(PROD1)
	#ifdef DELIVERY_APP
		#define ANG_SPEED_MUL 2500
		#define ANG_ACCEL 150
		#define ANG_SPEED_MAX 120000
	#else
		#define ANG_SPEED_MUL 3760
		#define ANG_ACCEL 220
		#define ANG_SPEED_MAX 360000
	#endif
#endif

#ifdef PULU1
#define ANG_SPEED_MUL 3900
#define ANG_SPEED_MAX 300000
#define ANG_ACCEL 250
#endif

void set_top_speed_ang(int speed) {
	int new_speed = speed * ANG_SPEED_MUL;
	if (new_speed < MIN_SPEED_ANG) {
		new_speed = MIN_SPEED_ANG;
	} else if (new_speed > ANG_SPEED_MAX) {
		new_speed = ANG_SPEED_MAX;
	}
	ang_top_speed = new_speed;
}

void set_top_speed_ang_max(int speed) {
	int new_speed = speed * ANG_SPEED_MUL;
	if (new_speed < MIN_SPEED_ANG) {
		new_speed = MIN_SPEED_ANG;
	} else if (new_speed > ANG_SPEED_MAX) {
		new_speed = ANG_SPEED_MAX;
	}
	if (new_speed < ang_top_speed) {
		ang_top_speed = new_speed;
	}
}

void set_top_speed_fwd(int speed) {
	int new_speed = speed * FWD_SPEED_MUL;
	if (new_speed < MIN_SPEED_FWD) {
		new_speed = MIN_SPEED_FWD;
	} else if (new_speed > 1200000) {
		new_speed = 1200000;
	}
	fwd_top_speed = new_speed;
}

void set_top_speed_fwd_max(int speed) {
	int new_speed = speed * FWD_SPEED_MUL;
	if (new_speed < MIN_SPEED_FWD) {
		new_speed = MIN_SPEED_FWD;
	} else if (new_speed > 1200000) {
		new_speed = 1200000;
	}
	if (new_speed < fwd_top_speed) {
		fwd_top_speed = new_speed;
	}
}

void set_top_speed(int speed) {
	set_top_speed_fwd(speed);
	set_top_speed_ang(speed);
}

void set_top_speed_max(int speed) {
	set_top_speed_fwd_max(speed);
	set_top_speed_ang_max(speed);
}

void reset_speed_limits(void) {
	set_top_speed(50);
}

void rotate_rel(int angle) {
	dbg_teleportation_bug(100);
	do_correct_fwd = false;
	do_correct_angle = false;
	feedback_stop_flags = 0;
	feedback_stop_param1 = 0;
	feedback_stop_param2 = 0;
	feedback_stop_param1_store = 0;
	feedback_stop_param2_store = 0;

	aim_angle += angle;

	speed_limit_lowered = 0;
	manual_control = 0;
	set_robot_moving();
	reset_wheel_slip_det = true;
	dbg_teleportation_bug(101);
}

void rotate_abs(int angle) {
	dbg_teleportation_bug(102);
	do_correct_fwd = false;
	do_correct_angle = false;
	feedback_stop_flags = 0;
	feedback_stop_param1 = 0;
	feedback_stop_param2 = 0;
	feedback_stop_param1_store = 0;
	feedback_stop_param2_store = 0;
	aim_angle = angle;

	speed_limit_lowered = 0;
	manual_control = 0;
	set_robot_moving();
	reset_wheel_slip_det = true;
	dbg_teleportation_bug(103);
}

void change_angle_abs(int angle) {
	aim_angle = angle;
}

void change_angle_rel(int angle) {
	aim_angle += angle;
}

void change_angle_to_cur(void) {
	aim_angle = cur_pos.ang;
}

void straight_rel(int fwd /*in mm*/) {
	dbg_teleportation_bug(104);

	do_correct_fwd = false;
	do_correct_angle = false;
	feedback_stop_flags = 0;
	feedback_stop_param1 = 0;
	feedback_stop_param2 = 0;
	feedback_stop_param1_store = 0;
	feedback_stop_param2_store = 0;
	speed_limit_lowered = 0;
	fwd_accel = final_fwd_accel / 4;
	aim_fwd = mm2Int(fwd);
	manual_control = 0;
	set_robot_moving();
	reset_wheel_slip_det = true;
	dbg_teleportation_bug(105);
}

int get_fwd(void) {
	return (Int2mm(aim_fwd));
}

void change_straight_rel(int fwd /*in mm*/) {
	aim_fwd = mm2Int(fwd);
}

void reset_movement(void) {
	aim_fwd = 0;
	aim_angle = cur_pos.ang;
	fwd_speed_limit = fwd_accel * 80; // use starting speed that equals to 80ms of acceleration
	reset_wheel_slip_det = true;
}

static bool auto_keepstill;

static bool angular_allowed = true;
static bool straight_allowed = true;

int accurate_turngo;

void allow_angular(bool yes) {
	angular_allowed = yes;
}

void allow_straight(bool yes) {
	straight_allowed = yes;
}

void auto_disallow(bool yes) {
	auto_keepstill = yes;
}

static int ang_idle = 1;
static int fwd_idle = 1;

int correcting_angle(void) {
	return do_correct_angle || (ang_idle < 700);
}

int get_ang_err(void) {
	return cur_pos.ang - aim_angle;
}

int angle_almost_corrected(void) {
	int ang_err = cur_pos.ang - aim_angle;
	if (ang_err > ANG_DEG(-5) && ang_err < ANG_DEG(5)) {
		return 1;
	}
	return 0;
}

int correcting_straight(void) {
	return do_correct_fwd || (fwd_idle < 700);
}

int correcting_either(void) {
	return do_correct_angle || (ang_idle < 700) || do_correct_fwd || (fwd_idle < 700);
}

void zero_angle(void) {
	aim_angle = 0;
	cur_pos.ang = 0;
	reset_wheel_slip_det = true;
}

void zero_coords(void) {
	cur_x = cur_y = 0;
	reset_wheel_slip_det = true;
}

// Gyro conversion multiplier
// too big = lidar drifts ccw on screen. too small = lidar drifts cw on screen
#ifdef RN1P4
int64_t gyro_mul_neg = 763300LL << 16;
int64_t gyro_mul_pos = 763300LL << 16;
#endif

#ifdef PULU1
int64_t gyro_mul_neg = 767116LL << 16;
int64_t gyro_mul_pos = 763300LL << 16;
#endif

#ifdef RN1P6
int64_t gyro_mul_neg = 763300LL << 16;
int64_t gyro_mul_pos = 763300LL << 16;
#endif

#ifdef RN1P7
int64_t gyro_mul_neg = 763300LL<<16;
int64_t gyro_mul_pos = 763300LL<<16;
#endif

#ifdef PROD1
int64_t gyro_mul_neg = 763300LL << 16;
int64_t gyro_mul_pos = 763300LL << 16;
#endif

int gyro_avgd = 0;

/*
Internal correction function stub: not being used right now. Includes code to self-calibrate gyro.
This function was once used when we did lidar scan matching on MCU. Now there is no use, all corrections
to coords are external from the Raspi.

void correct_location_without_moving(pos_t corr) {
	dbg_teleportation_bug(106);

	cur_x += corr.x << 16;
	cur_y += corr.y << 16;
	cur_pos.ang += corr.ang;
	aim_angle += corr.ang;

	if (gyro_avgd < -300) {
		gyro_mul_neg += corr.ang;
	} else if (gyro_avgd > 300) {
		gyro_mul_pos += corr.ang;
	} else {
		gyro_mul_neg += corr.ang >> 1;
		gyro_mul_pos += corr.ang >> 1;
	}
	dbg_teleportation_bug(107);

}
*/

void correct_location_without_moving_external(pos_t corr) {
	dbg_teleportation_bug(108);

	if (corr.x < -2000 || corr.x > 2000 || corr.y < -2000 || corr.y > 2000) {
		return;
	}

	dbg_teleportation_bug(109);

	__disable_irq();
	cur_x += mm2Int(corr.x);
	cur_y += mm2Int(corr.y);
	cur_pos.ang += corr.ang;
	aim_angle += corr.ang;

	if (corr.ang < 0) {
		gyro_mul_neg -= corr.ang >> 1;
		gyro_mul_pos += corr.ang >> 2;
	} else {
		gyro_mul_neg -= corr.ang >> 2;
		gyro_mul_pos += corr.ang >> 1;
	}

	__enable_irq();

//	dbg[7] = (int32_t) (gyro_mul_neg >> 16);
//	dbg[8] = (int32_t) (gyro_mul_pos >> 16);

	static int avg_corr_flt;

	avg_corr_flt = (corr.ang + 31 * avg_corr_flt) >> 5; // moving average for debug purposes
//	dbg[9] = avg_corr_flt / ANG_0_01_DEG;

	dbg_teleportation_bug(110);

}

void set_location_without_moving_external(pos_t new_pos) {
	dbg_teleportation_bug(111);

	__disable_irq();
	cur_x = mm2Int(new_pos.x);
	cur_y = mm2Int(new_pos.y);
	cur_pos.ang = new_pos.ang;
	aim_angle = new_pos.ang;
	reset_wheel_slip_det = true;
	__enable_irq();
	dbg_teleportation_bug(112);

}

volatile int compass_round_on;

void sync_to_compass(void) {
	cur_pos.ang = aim_angle = cur_compass_angle;
}

void compass_fsm(int cmd) {
	static int compass_x_min = 0;
	static int compass_x_max = 0;
	static int compass_y_min = 0;
	static int compass_y_max = 0;

	static int state = 0;

	dbg_teleportation_bug(113);

	if (cmd == 1 && state == 0) {
		state = 1;
	}

	if (state) {
		compass_round_on = 1;
	} else {
		compass_round_on = 0;
	}
	int cx = latest_compass->x;
	int cy =
#ifdef RN1P4
			latest_compass->y;
#endif
#if defined(PULU1) || defined(RN1P6) || defined(RN1P7) || defined(PROD1)
			-1 * latest_compass->y;
#endif

	int ang_err = cur_pos.ang - aim_angle;
	if (state == 1) {
		auto_disallow(true);
		allow_angular(true);
		allow_straight(false);
		compass_x_min = compass_x_max = cx;
		compass_y_min = compass_y_max = cy;

		set_top_speed_ang(25);
		rotate_rel(ANG_DEG(120));

		state = 2;
	} else if (state >= 2 && state < 7) {
		if (ang_err > ANG_DEG(-60) && ang_err < ANG_DEG(60)) {
			set_top_speed_ang(25);
			rotate_rel(ANG_DEG(120));
			state++;
		}
	} else if (state == 7) {
		state = 0;
		reset_wheel_slip_det = true;
		sync_to_compass();
	}

	/*	Compass algorithm

		To compensate for robot-referenced magnetic fields and offset errors:
		Track max, min readings on both X, Y axes while the robot is turning.
		Scale readings so that they read zero on the middle of the range, e.g.,
		if X axis reads between 1000 and 3000, make 2000 read as 0.
		Then calculate the angle with arctan (atan2() handles the signs to resolve
		the correct quadrant).
	*/

	if (cx < compass_x_min) {
		compass_x_min = cx;
	}
	if (cy < compass_y_min) {
		compass_y_min = cy;
	}
	if (cx > compass_x_max) {
		compass_x_max = cx;
	}
	if (cy > compass_y_max) {
		compass_y_max = cy;
	}
	int dx = compass_x_max - compass_x_min;
	int dy = compass_y_max - compass_y_min;
	if (dx > 500 && dy > 500) {
		int dx2 = compass_x_max + compass_x_min;
		int dy2 = compass_y_max + compass_y_min;
		cx = cx - dx2 / 2;
		cy = cy - dy2 / 2;

		double heading = atan2(cx, cy) + M_PI;
		if (heading < -1.0 * M_PI) {
			heading += 2.0 * M_PI;
		} else if (heading > 1.0 * M_PI) {
			heading -= 2.0 * M_PI;
		}
		heading /= (2.0 * M_PI);
		heading *= -65536.0 * 65536.0;
		cur_compass_angle = (int) heading;
	}
	dbg_teleportation_bug(114);
}

void move_arc_manual(int comm, int ang) {
	manual_common_speed = comm << 5;
	manual_ang_speed = ang << 5;
	manual_control = 1;
	set_robot_moving();
}

void unexpected_movement_detected(int param1, int param2) {
	// Store in case we have a gyro (wheel slip) stop, these can be useful to analyze the first hit.
	feedback_stop_param1_store = param1;
	feedback_stop_param2_store = param2;
	lidar_mark_invalid();
}

void collision_detected(int reason, int param1, int param2) {
	dbg_teleportation_bug(115);

	if (!feedback_stop_flags) {
		if (reason != 1) {
			feedback_stop_param1 = feedback_stop_param1_store;
			feedback_stop_param2 = feedback_stop_param2_store;
		} else {
			feedback_stop_param1 = param1;
			feedback_stop_param2 = param2;
		}
		feedback_stop_flags = reason;
		lidar_mark_invalid();
		reset_movement();
		stop_navig_fsms();
	}
	dbg_teleportation_bug(116);
}

int coll_det_on;
void enable_collision_detection(void) {
	coll_det_on = 1;
}

// Run this at 1 kHz
void run_feedbacks(uint32_t sens_status) {
	static int fwd_nonidle;
	static int init_wait_cnt = 100;
	int i;
	static int cnt = 0;
	static int prev_gyro_cnt = 0;
	static int prev_xcel_cnt = 0;
	static int speeda = 0;
	static int speedb = 0;
	static int ang_speed = 0;
	static int fwd_speed = 0;
	static int expected_fwd_accel = 0;
	static int16_t prev_wheel_counts[2];
	static int rear_wheels_in_line = 0;

	cnt++;

	dbg_teleportation_bug(120);

	if (robot_nonmoving_cnt > 600) {	// Is robot moving?
		robot_nonmoving = 1;
	} else {
		robot_nonmoving = 0;
		robot_nonmoving_cnt++;
	}

	int ang_err = cur_pos.ang - aim_angle;
	int fwd_err = aim_fwd;
//do_correct_fwd?
	if (straight_allowed && ((fwd_err < mm2Int(-40)) || fwd_err > mm2Int(40))) { //|fwd_err| >40mm
		if (accurate_turngo) {
			if (ang_err > ANG_DEG(-3) && ang_err < ANG_DEG(3)) {						//|ang_err| < 3deg
				do_correct_fwd = true;
			} else if (ang_err < ANG_DEG(-5) && ang_err > ANG_DEG(5)) {				//|ang_err| > 5deg
				do_correct_fwd = false;
			}
		} else {
			if (ang_err > ANG_DEG(-15) && ang_err < ANG_DEG(15)) {					//|ang_err| < 15deg
				do_correct_fwd = true;
			} else if (ang_err < ANG_DEG(-30) && ang_err > ANG_DEG(30)) {			//|ang_err| > 30deg
				do_correct_fwd = false;
			}
		}
	}
	if (!straight_allowed || (fwd_err > mm2Int(-15) && fwd_err < mm2Int(15))) {	//|fwd_err| <15mm
		do_correct_fwd = false;
	}

	/*
	When going straight at the same time, smaller error in angle starts the correction.
	This causes the following style:
	- When turning first, angle is only fixed within +-3 deg
	- When the straight segment starts, angle is being fixed within +-0.5 deg.
	- This way, turning is quick, and there is no oscillation caused by the BLDC control at extremely small speeds / steps.
	- When the robot moves forward at the same time, tire speeds have common mode, and angular control only slighly changes the offset between the tires.
	  This works very well without oscillation. Also, during straight segment, the rear wheels are properly aligned.
	- Start doing the tighter angular control after 300 ms of straight segment
	*/

   dbg_teleportation_bug(121);

	if (accurate_turngo) {

		if (angular_allowed
				&& ((fwd_nonidle > 300 && (ang_err < (-ANG_0_5_DEG ) || ang_err > ANG_0_5_DEG ))
						|| (ang_err < ANG_DEG(-1) || ang_err > ANG_DEG(1)))) {
			do_correct_angle = true;
		}

		if (!angular_allowed || (fwd_nonidle > 300 && (ang_err > (-ANG_0_25_DEG ) && ang_err < (ANG_0_25_DEG )))
				|| ((ang_err > (-ANG_0_5_DEG ) && ang_err < (ANG_0_5_DEG )))) {
			do_correct_angle = false;
		}

	} else {
		if (angular_allowed
				&& ((fwd_nonidle > 300 && (ang_err < (-ANG_0_5_DEG ) || ang_err > ANG_0_5_DEG ))
						|| (ang_err < ANG_DEG(-5) || ang_err > ANG_DEG(5)))) {
			do_correct_angle = true;
		}

		if (!angular_allowed || (fwd_nonidle > 300 && (ang_err > (-ANG_0_25_DEG ) && ang_err < (ANG_0_25_DEG )))
				|| ((ang_err > ANG_DEG(-3) && ang_err < ANG_DEG(3)))) {
			do_correct_angle = false;
		}
	}

	if (ang_err > ANG_DEG(-10) && ang_err < ANG_DEG(10) && !fwd_idle) {
		rear_wheels_in_line++;
	} else {
		rear_wheels_in_line = 0;
	}
	if (rear_wheels_in_line > 800) {
		fwd_accel = final_fwd_accel;
	}

	if (auto_keepstill) {
		if (!do_correct_angle && ang_idle > 300 && !do_correct_fwd && fwd_idle > 300) {
			angular_allowed = false;
			straight_allowed = false;
		}
	}

	dbg_teleportation_bug(122);

	if (!manual_control && do_correct_angle) {
		if (ang_idle) {
			ang_idle = 0;
			ang_speed_limit = ANG_ACCEL * 20; // use starting speed that equals to 20ms of acceleration
		}

		// Calculate angular speed with P loop from the gyro integral.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if (ang_speed_limit < ang_top_speed) {
			ang_speed_limit += ANG_ACCEL;
		} else if (ang_speed_limit > ang_top_speed + ANG_ACCEL + 1) {
			ang_speed_limit -= 2 * ANG_ACCEL;
		} else {
			ang_speed_limit = ang_top_speed;
		}
		int new_ang_speed = (ang_err >> 20) * ang_p; /*step-style feedforward for minimum speed*/
		if (ang_err > 0) {
			new_ang_speed += STEP_FEEDFORWARD_ANG;
		} else {
			new_ang_speed += -STEP_FEEDFORWARD_ANG;
		}

		if (new_ang_speed < 0 && new_ang_speed < -1 * ang_speed_limit) {
			new_ang_speed = -1 * ang_speed_limit;
		}
		if (new_ang_speed > 0 && new_ang_speed > ang_speed_limit) {
			new_ang_speed = ang_speed_limit;
		}
		ang_speed = new_ang_speed;

#ifdef PCB1B
		if (ang_err > ANG_DEG(4)) {	//leds control
			leds_motion_blink_left = 1;
			leds_motion_blink_right = 0;
		} else if (ang_err < ANG_DEG(-4)) {
			leds_motion_blink_left = 0;
			leds_motion_blink_right = 1;
		}
#endif
	} else {
#ifdef PCB1B
		leds_motion_blink_left = 0;
		leds_motion_blink_right = 0;
#endif
		if (ang_idle < 10000) {
			ang_idle++;
		}
		ang_speed = 0;
	}

	static int32_t prev_cur_ang = 0;

	int16_t wheel_counts[2];
	wheel_counts[0] = motcon_rx[A_MC_IDX].pos;	//counts of passed Halls
	wheel_counts[1] = motcon_rx[B_MC_IDX].pos;

	if (init_wait_cnt) {
		init_wait_cnt--;
		prev_wheel_counts[0] = wheel_counts[0];
		prev_wheel_counts[1] = wheel_counts[1];
		prev_cur_ang = cur_pos.ang;
		reset_wheel_slip_det = true;
	}
	int16_t wheel_deltas[2] = { wheel_counts[0] - prev_wheel_counts[0], wheel_counts[1] - prev_wheel_counts[1] };
	wheel_deltas[1] *= -1;	//So, the wheel_counts are same symbol.

	dbg_teleportation_extra.wd0 = wheel_deltas[0];
	dbg_teleportation_extra.wd1 = wheel_deltas[1];

	prev_wheel_counts[0] = wheel_counts[0];
	prev_wheel_counts[1] = wheel_counts[1];

	if (wheel_deltas[0] != 0 || wheel_deltas[1] != 0) {
		set_robot_moving();
	}
	int movement;
	if (wheel_deltas[0] < -20 || wheel_deltas[0] > 20 || wheel_deltas[1] < -20 || wheel_deltas[1] > 20) {
		movement = 0;
	} else {
		// in 1mm/65536:
		movement = ((int32_t) wheel_deltas[0] + (int32_t) wheel_deltas[1]) *
#if defined(RN1P4) || defined(RN1P7) || defined(PROD1)
				278528; //L=(l+r)*k/2, k= mm per pulse=8.5 mm/p* 65536
#endif
#ifdef RN1P6
		282242; //2*4.306671142578125mm/p
#endif
#ifdef PULU1
		246147; //2*3.7559051513671875 mm/p
#endif
	}
	dbg_teleportation_extra.movement = movement;

	int turned_by_wheels = (wheel_deltas[0] - wheel_deltas[1]) *
#if defined(RN1P4) || defined(RN1P6) || defined(RN1P7) || defined(PROD1)
			15500000;	//omega=(l-r)*k/base= mm*p/base*RAD2nDEG base=374.86mm, K=8.5*RAD2nDEG/374.86=15499933
#endif
#ifdef PULU1
	21678321; //base=236.864mm
#endif

	int turned_by_gyro = cur_pos.ang - prev_cur_ang;
	int turn_wheels_gyro_err = turned_by_wheels - turned_by_gyro;

	dbg_teleportation_bug(123);

	// Keep track of integral of error between gyro and wheel-based turning information.
	// Decrement the error integral slowly (2 deg per s) to prevent small error buildup.
	// Use error signal to detect wheel slip.
	static int turn_wheels_gyro_err_integral = 0;
	turn_wheels_gyro_err_integral += turn_wheels_gyro_err;

	if (turn_wheels_gyro_err_integral < 0) {
		turn_wheels_gyro_err_integral += 4 * ANG_0_001_DEG;	//1kHz=1ms
	} else {
		turn_wheels_gyro_err_integral -= 4 * ANG_0_001_DEG;
	}
	prev_cur_ang = cur_pos.ang;

	if (reset_wheel_slip_det) {
		reset_wheel_slip_det = false;
		turn_wheels_gyro_err_integral = 0;
	} else {
		if (turn_wheels_gyro_err_integral < ANG_DEG(-12)) {
			collision_detected(do_correct_fwd ? 2 : 6, 0, 0);
			turn_wheels_gyro_err_integral = 0;
		} else if (turn_wheels_gyro_err_integral > ANG_DEG(12)) {
			collision_detected(do_correct_fwd ? 3 : 5, 0, 0);
			turn_wheels_gyro_err_integral = 0;
		}
	}

	aim_fwd -= movement;

	int y_idx = cur_pos.ang >> SIN_LUT_SHIFT;	//12bits=4096
	if (y_idx < 0) {
		y_idx += SIN_LUT_POINTS;
	} else if (y_idx >= SIN_LUT_POINTS) {
		y_idx -= SIN_LUT_POINTS;
	}
	int x_idx = SIN_LUT_POINTS / 4 - y_idx;
	if (x_idx < 0) {
		x_idx += SIN_LUT_POINTS;
	} else if (x_idx >= SIN_LUT_POINTS) {
		x_idx -= SIN_LUT_POINTS;
	}
	dbg_teleportation_bug(124);

	dbg_teleportation_extra.y_idx = y_idx;
	dbg_teleportation_extra.x_idx = x_idx;

	dbg_teleportation_extra.x_before = cur_x;
	dbg_teleportation_extra.y_before = cur_y;

	cur_x += Int2mm((int64_t ) sin_lut[x_idx] * (int64_t ) movement) * 2;
	cur_y += Int2mm((int64_t ) sin_lut[y_idx] * (int64_t ) movement) * 2;	//>> 15;

	dbg_teleportation_extra.x_after = cur_x;
	dbg_teleportation_extra.y_after = cur_y;

	dbg_teleportation_extra.dx = Int2mm((int64_t ) sin_lut[x_idx] * (int64_t ) movement) * 2;	//>> 15;
	dbg_teleportation_extra.dy = Int2mm((int64_t ) sin_lut[y_idx] * (int64_t ) movement) * 2;	//>> 15;

	int tmp_expected_accel = -1 * fwd_speed;

	dbg_teleportation_bug(125);

	if (!manual_control && do_correct_fwd) {
		if (fwd_idle) {
			fwd_idle = 0;
		}
		fwd_nonidle++;

		int top_speed = fwd_top_speed;
		if (ang_err < ANG_DEG(-30) || ang_err > ANG_DEG(30)) {
			top_speed = 0;
		} else if (ang_err < ANG_DEG(-25) || ang_err > ANG_DEG(25)) {
			if (top_speed > 30000) {
				top_speed = 30000;
			}
		} else if (ang_err < ANG_DEG(-20) || ang_err > ANG_DEG(20)) {
			if (top_speed > 60000) {
				top_speed = 60000;
			}
		} else if (ang_err < ANG_DEG(-15) || ang_err > ANG_DEG(15)) {
			if (top_speed > 150000) {
				top_speed = 150000;
			}
		} else if (ang_err < ANG_DEG(-10) || ang_err > ANG_DEG(10)) {
			if (top_speed > 300000) {
				top_speed = 300000;
			}
		}

		// Calculate linear speed with P loop from position error.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if (fwd_speed_limit < top_speed) {
			fwd_speed_limit += fwd_accel;
		} else if (fwd_speed_limit > top_speed + fwd_accel + 1) {
			fwd_speed_limit -= fwd_accel;
		} else {
			fwd_speed_limit = top_speed;
		}
		int new_fwd_speed = Int2mm(fwd_err) * fwd_p
						+ ((fwd_err > 0) ? STEP_FEEDFORWARD_FWD : (-STEP_FEEDFORWARD_FWD)) /*step-style feedforward for minimum speed*/;
		if (new_fwd_speed < 0 && new_fwd_speed < -1 * fwd_speed_limit) {
			new_fwd_speed = -1 * fwd_speed_limit;
		}
		if (new_fwd_speed > 0 && new_fwd_speed > fwd_speed_limit) {
			new_fwd_speed = fwd_speed_limit;
		}
		fwd_speed = new_fwd_speed;
#ifdef PCB1B
		if (new_fwd_speed > 0) {
			leds_motion_forward = 1;
		} else {
			leds_motion_forward = 0;
		}
#endif
	} else {
#ifdef PCB1B
		leds_motion_forward = 0;
#endif
		if (fwd_idle < 10000) {
			fwd_idle++;
		}
		fwd_nonidle = 0;
		if (fwd_speed) {
			fwd_speed -= 2 * fwd_accel;
			if (fwd_speed < 0) {
				fwd_speed = 0;
			}
		}
	}

	if (ang_speed != 0 || fwd_speed != 0) {
		set_robot_moving();
	}
	tmp_expected_accel += fwd_speed;
	expected_fwd_accel = ((tmp_expected_accel << 16) + 63 * expected_fwd_accel) >> 6;

	if (sens_status & GYRO_NEW_DATA) {
		int latest[3] =
#ifdef RN1P4
				{	latest_gyro->x, latest_gyro->y, latest_gyro->z};
#endif
#if defined(PULU1) || defined(RN1P6) || defined(RN1P7) || defined(PROD1)
				{ latest_gyro->x, latest_gyro->y, -1 * latest_gyro->z };
		// todo: check what needs to be done with x and y, currently not used for anything except motion detection thresholding
#endif

#define GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_X 3000
#define GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Y 3000
#define GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Z 800

#define GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_X 2000
#define GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Y 2000
#define GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Z 400

		static int gyro_thresh_cnt = 0;

		if(latest[0] < -GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_X || latest[0] > GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_X ||
		   latest[1] < -GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Y || latest[1] > GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Y ||
		   latest[2] < -GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Z || latest[2] > GYRO_RAW_MOVEMENT_DETECT_THRESHOLD_Z) {
			if (++gyro_thresh_cnt > 2) {
				set_robot_moving();
				dbg[6]++;
			}
		} else {
			gyro_thresh_cnt = 0;
		}

#define GYRO_DC_CORRS_VALID_LIM 1000
		static int gyro_dc_corrs_valid;
		if (robot_nonmoving) { // Moving average or Complementary filter, alpha=0.00390625.
			//new=latest*65536/2, a=(new+a*255)/256=new*0.00390625+a*0.99609375;
			gyro_dc_corrs[0] = (Ang2Int32(latest[0]) / 2 + 255 * gyro_dc_corrs[0]) / 256;
			gyro_dc_corrs[1] = (Ang2Int32(latest[1]) / 2 + 255 * gyro_dc_corrs[1]) >> 8;
			gyro_dc_corrs[2] = (Ang2Int32(latest[2]) / 2 + 255 * gyro_dc_corrs[2]) >> 8;
			if (gyro_dc_corrs_valid < GYRO_DC_CORRS_VALID_LIM) {
				gyro_dc_corrs_valid++;
			}
		}

		int gyro_dt = cnt - prev_gyro_cnt;
		prev_gyro_cnt = cnt;
		// Gyro should give data at 200Hz, which is 5 steps at 1kHz
		if (gyro_dt < 2 || gyro_dt > 20) {
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			gyro_dt = 5;
			gyro_timing_issues++;
		}

		static int dbgcnt = 0;
		static int dbgmax = -999999;
		static int dbgmin = 999999;
		static int dbgavg = 0;
		static int dbgavg_cnt = 0;

		/*
			Some gyro results

			Steady state, room temp

				001a85	jkl

			min	-304	-70
			max	-27	219
			avg	-160	78

		*/

		dbg[0] = gyro_timing_issues;
		if (++dbgcnt == 200 * 5) { // 5 seconds
			dbgcnt = 0;
			dbg[1] = dbgmin;
			dbg[2] = dbgmax;
			dbg[3] = dbgavg_cnt;
			dbg[4] = dbgavg / dbgavg_cnt;
			dbg[5] = Int32toAng(gyro_dc_corrs[0])*2;

			dbgmax = -999999;
			dbgmin = 999999;
			dbgavg = 0;
			dbgavg_cnt = 0;
		}

		if (latest[0] > dbgmax) {
			dbgmax = latest[0];
		}
		if (latest[0] < dbgmin) {
			dbgmin = latest[0];
		}
		dbgavg += latest[0];
		dbgavg_cnt++;

		for (i = 0; i < 3; i++) {	//Int
			latest[i] -= Int32toAng(gyro_dc_corrs[i])*2;
			latest[i] *= gyro_dt;
			gyro_long_integrals[i] += latest[i];
			gyro_short_integrals[i] += latest[i];
		}

		static int gyro_dccomp_thresh_cnt = 0;
		if(gyro_dc_corrs_valid == GYRO_DC_CORRS_VALID_LIM) {
			if(latest[0] < -GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_X || latest[0] > GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_X ||
			   latest[1] < -GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Y || latest[1] > GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Y ||
			   latest[2] < -GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Z || latest[2] > GYRO_DCCOMP_MOVEMENT_DETECT_THRESHOLD_Z)	{
				if (++gyro_dccomp_thresh_cnt > 2) {
					set_robot_moving();
					dbg[7]++;
				}
			} else {
				gyro_dccomp_thresh_cnt = 0;
			}
		}

		//1 gyro unit = 7.8125 mdeg/s; integrated at 1kHz timesteps, 1 unit = 7.8125 udeg
		// Correct ratio = (7.8125*10^-6)/(360/(2^32)) = 93.2067555555589653990
		// Approximated ratio = 763550/8192  = 763550>>13 = 93.20678710937500
		// Corrected empirically from there.

		 // Prevent slow gyro drifting during no operation
//		int gyro_blank = robot_nonmoving ? (40 * 5) : (0);
//		int gyro_blank = robot_nonmoving ? (100 * 5) : (50 * 5);
		int gyro_blank = robot_nonmoving ? (800 * 5) : (200 * 5);
//a=latest[2] << 8, gyro's unit is 256.
		gyro_avgd = ((latest[2] << 8) + 7 * gyro_avgd) >> 3; //Complementary filter, a=0.125*new+(1-0.125)*a

		// Don't let any higher-priority ISR read cur_pos while
		__disable_irq();
		// Copy accurate accumulation variables to cur_pos here:
		cur_pos.x = Int2mm(cur_x);		// >> 16;
		cur_pos.y = Int2mm(cur_y);		// >> 16;
		if (latest[2] < -1 * gyro_blank) {  // Negative = left
			cur_pos.ang += ((int64_t) latest[2] * (int64_t) Int32toAng(gyro_mul_neg)) >> 13; //drift correct.
		} else if (latest[2] > gyro_blank) {  // Positive = right
			cur_pos.ang += ((int64_t) latest[2] * (int64_t) Int32toAng(gyro_mul_pos)) >> 13;
		}
		__enable_irq();
	}

   dbg_teleportation_bug(126);

#ifdef USE_XCEL
	if (sens_status & XCEL_NEW_DATA) {
		int latest[3] =
#ifdef RN1P4
				{ latest_xcel->x, latest_xcel->y, latest_xcel->z };
#endif
#if defined(PULU1) || defined(RN1P6) || defined(RN1P7) || defined(PROD1)
				{ latest_xcel->x, latest_xcel->y, -1 * latest_xcel->z }; // todo: fix x, y
#endif

		static int xcel_flt[2];

//		if(robot_nonmoving) {
		xcel_dc_corrs[0] = ((latest[0] << 15) + 255 * xcel_dc_corrs[0]) >> 8;
		xcel_dc_corrs[1] = ((latest[1] << 15) + 255 * xcel_dc_corrs[1]) >> 8;
		xcel_dc_corrs[2] = ((latest[2] << 15) + 255 * xcel_dc_corrs[2]) >> 8;
//		}

		int xcel_dt = cnt - prev_xcel_cnt;
		prev_xcel_cnt = cnt;
		// Xcel should give data at 200Hz, which is 5 steps at 1kHz
		if (xcel_dt < 2 || xcel_dt > 20) {
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			xcel_dt = 5;
			xcel_timing_issues++;
		}

		for (i = 0; i < 3; i++) {
			latest[i] -= xcel_dc_corrs[i] >> 15;
			latest[i] *= xcel_dt;
			if (latest[i] < -3000 || latest[i] > 3000) {
				xcel_long_integrals[i] += (int64_t) latest[i];
			}
			xcel_short_integrals[i] += (int64_t) latest[i];
		}

		// Moving average 31/32
		xcel_flt[0] = ((latest[0] << 8) + 31 * xcel_flt[0]) >> 5;
		xcel_flt[1] = ((latest[1] << 8) + 31 * xcel_flt[1]) >> 5;

		if (coll_det_on && (xcel_flt[0] < XCEL_X_NEG_WARN
				|| xcel_flt[0] > XCEL_X_POS_WARN
				|| xcel_flt[1] < XCEL_Y_NEG_WARN
				|| xcel_flt[1] > XCEL_Y_POS_WARN)) {
			unexpected_movement_detected(xcel_flt[0] >> 8, xcel_flt[1] >> 8);
		}
		if (coll_det_on && (xcel_flt[0] < XCEL_X_NEG_COLL
						|| xcel_flt[0] > XCEL_X_POS_COLL
						|| xcel_flt[1] < XCEL_Y_NEG_COLL
						|| xcel_flt[1] > XCEL_Y_POS_COLL)) {
			collision_detected(1, xcel_flt[0] >> 8, xcel_flt[1] >> 8);
		}
//		int unexpected_accel = /*(expected_fwd_accel>>4)*/ 0 - latest[1];
		//1 xcel unit = 0.061 mg = 0.59841 mm/s^2; integrated at 1kHz timesteps, 1 unit = 0.59841 mm/s
	}
#endif

	int common_speed;
	if (manual_control) {
		speeda = -1 * manual_ang_speed;
		speedb = manual_ang_speed;
		reset_movement();
		stop_navig_fsms();
		set_robot_moving(); // to prevent surprises when auto mode goes back up.
		common_speed = manual_common_speed;
	} else {
		speeda = -1 * (ang_speed >> 8);
		speedb = (ang_speed >> 8);
		common_speed = fwd_speed >> 8;
	}

   dbg_teleportation_bug(127);

	if (speeda > MAX_DIFFERENTIAL_SPEED * 256) {
		speeda = MAX_DIFFERENTIAL_SPEED * 256;
	} else if (speeda < -MAX_DIFFERENTIAL_SPEED * 256) {
		speeda = -MAX_DIFFERENTIAL_SPEED * 256;
	}
	if (speedb > MAX_DIFFERENTIAL_SPEED * 256) {
		speedb = MAX_DIFFERENTIAL_SPEED * 256;
	} else if (speedb < -MAX_DIFFERENTIAL_SPEED * 256) {
		speedb = -MAX_DIFFERENTIAL_SPEED * 256;
	}
	int a = (common_speed) + (speeda);
	int b = (common_speed) + (speedb);

	if (a > MAX_SPEED) {
		a = MAX_SPEED;
	} else if (a < -MAX_SPEED) {
		a = -MAX_SPEED;
	}

	if (b > MAX_SPEED) {
		b = MAX_SPEED;
	} else if (b < -MAX_SPEED) {
		b = -MAX_SPEED;
	}
	if (robot_is_in_charger) {
		motcon_tx[A_MC_IDX].cur_limit = motcon_tx[B_MC_IDX].cur_limit = 4000; // as long as the speed is low, 4A motor current times 2 shouldn't blow the 3A power supply input fuse. (5000 was tested)

		motcon_tx[A_MC_IDX].res3 = motcon_tx[B_MC_IDX].res3 = ((uint16_t) mc_pid_imax << 8) | (uint16_t) mc_pid_feedfwd;
		motcon_tx[A_MC_IDX].res4 = motcon_tx[B_MC_IDX].res4 = ((uint16_t) (mc_pid_p >> 1) << 8) | (uint16_t) mc_pid_i; // lower P for more careful reaction
		motcon_tx[A_MC_IDX].res5 = motcon_tx[B_MC_IDX].res5 = ((uint16_t) mc_pid_d << 8);
	} else {
		motcon_tx[A_MC_IDX].cur_limit = motcon_tx[B_MC_IDX].cur_limit = 24000;

		motcon_tx[A_MC_IDX].res3 = motcon_tx[B_MC_IDX].res3 = ((uint16_t) mc_pid_imax << 8) | (uint16_t) mc_pid_feedfwd;
		motcon_tx[A_MC_IDX].res4 = motcon_tx[B_MC_IDX].res4 = ((uint16_t) mc_pid_p << 8) | (uint16_t) mc_pid_i;
		motcon_tx[A_MC_IDX].res5 = motcon_tx[B_MC_IDX].res5 = ((uint16_t) mc_pid_d << 8);
	}

//	dbg[0] = motcon_rx[A_MC_IDX].current;
//	dbg[1] = motcon_rx[A_MC_IDX].cur_limit_mul;
//	dbg[2] = motcon_rx[A_MC_IDX].num_hard_limits;

//	dbg[3] = motcon_rx[B_MC_IDX].current;
//	dbg[4] = motcon_rx[B_MC_IDX].cur_limit_mul;
//	dbg[5] = motcon_rx[B_MC_IDX].num_hard_limits;

//	dbg[5] = motcon_tx[A_MC_IDX].cur_limit;

	if (host_alive_watchdog) {
		host_alive_watchdog--;
		motcon_tx[A_MC_IDX].state = 5;
		motcon_tx[B_MC_IDX].state = 5;
		motcon_tx[A_MC_IDX].speed = a;
		motcon_tx[B_MC_IDX].speed = -1 * b;
	} else {
		motcon_tx[A_MC_IDX].state = 1;
		motcon_tx[B_MC_IDX].state = 1;
		motcon_tx[A_MC_IDX].speed = 0;
		motcon_tx[B_MC_IDX].speed = 0;
		feedback_stop_flags = 4;
		reset_movement();
		stop_navig_fsms(); // to prevent surprises when we are back up.
	}

	dbg_teleportation_bug(128);
}

volatile int dbg_teleportation_bug_report;

volatile dbg_teleportation_bug_data_t dbg_teleportation_bug_data;
volatile dbg_teleportation_extra_t dbg_teleportation_extra;
volatile dbg_teleportation_extra_t dbg_teleportation_extra_to_send;

/*
	Okay, we have had an interesting bug where the robot coordinates corrupt after many hours of runtime.
	It appears rarely enough that it's hard to debug. Not even 100% sure it happens on MCU. Anyway, this
	function should help narrow it down. Just call this in convenient locations (marked with different ids),
	and if the coords change abruptly, stack of the ids is stored and can be sent.
*/
void dbg_teleportation_bug(int id) {
	int64_t x, y;
	__disable_irq();
	x = cur_x;
	y = cur_y;

	static int64_t prev_x, prev_y;
	static int32_t prev_id, prev2_id, prev3_id, prev4_id;

	int64_t dx = x - prev_x;
	int64_t dy = y - prev_y;

	if (dx < (-2000LL << 16) || dx > (2000LL << 16) || dy < (-2000LL << 16) || dy > (2000LL << 16)) {
		if (dbg_teleportation_bug_report == 0) {
			dbg_teleportation_bug_report = 1;
			dbg_teleportation_bug_data.id = id;
			dbg_teleportation_bug_data.prev_id = prev_id;
			dbg_teleportation_bug_data.prev2_id = prev2_id;
			dbg_teleportation_bug_data.prev3_id = prev3_id;
			dbg_teleportation_bug_data.prev4_id = prev4_id;
			dbg_teleportation_bug_data.prev_x = prev_x;
			dbg_teleportation_bug_data.prev_y = prev_y;
			dbg_teleportation_bug_data.cur_x = x;
			dbg_teleportation_bug_data.cur_y = y;
//			memcpy(dbg_teleportation_extra_to_send, dbg_teleportation_extra, sizeof dbg_teleportation_extra_t);
		}
	}

	prev_x = x;
	prev_y = y;

	prev4_id = prev3_id;
	prev3_id = prev2_id;
	prev2_id = prev_id;
	prev_id = id;

	__enable_irq();

}

