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

#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>
#include "feedbacks.h" // for pos_t

typedef struct
{
	int valid;
	int32_t x;
	int32_t y;
} point_t;

typedef struct
{
	int32_t x;
	int32_t y;
} xy_i32_t;

typedef struct
{
	int16_t x;
	int16_t y;
} xy_i16_t;


#if defined(RN1P4) || defined(RN1P6) || defined(RN1P7)
	#define LIDAR_IGNORE_LEN 350 // mm, everything below this is marked in ignore list during ignore scan.
	#define LIDAR_IGNORE_LEN_FRONT 200 // mm, everything below this is marked in ignore list during ignore scan.
#endif
#ifdef PULU1
	#define LIDAR_IGNORE_LEN 250
	#define LIDAR_IGNORE_LEN_FRONT 120
#endif

/*
Lidar scan to be transferred to the host computer.
Keep 32-bit alignment so that the samples are quick to access.
Make sure the alignment doesn't break when adding/removing variables.



*/

#define LIDAR_MAX_POINTS 720

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint8_t status;
	uint8_t id;
	int16_t n_points;
	pos_t pos_at_start;
	pos_t pos_at_end;

	/*
		All scan points are referenced to refxy instead of the world origin.
		This has the following benefits:
		- due to the data size reduction needed for UART output, this was done anyway (32b->16b) - now we can send the raw struct as is!
		- when processing the data in the scan matching algorithm, 16-bit data allows much faster load + SIMD operations

		refxy may be, but usually isn't the robot pose at any point during the scan, but it's near due to the +/- 32 meter radius limit of 16 bits.

		As scan[] is the last thing in this struct, you can always truncate when sending or copying: use LIDAR_SIZEOF(lidar_scan) macro to get the actual
		required size for the struct.
	*/
	xy_i32_t refxy;
	xy_i16_t scan[LIDAR_MAX_POINTS];
} lidar_scan_t;

#define LIDAR_SIZEOF(scan) (1+1+2+2*sizeof(pos_t)+sizeof(xy_i32_t)+sizeof(xy_i16_t)*((scan).n_points))

extern lidar_scan_t lidar_scans[2];
extern lidar_scan_t *acq_lidar_scan;
extern lidar_scan_t *prev_lidar_scan;

extern int lidar_cur_n_samples;

void send_lidar_to_uart(lidar_scan_t* in, int significant_for_mapping);


typedef enum
{
	S_LIDAR_UNINIT			= 0,
	S_LIDAR_OFF			= 1,
	S_LIDAR_WAITPOWERED		= 2,
	S_LIDAR_PRECONF_WAIT_READY	= 3, // Quite stupidly, we need to poll whether the motor has reached its initial (default, or the previous) setpoint, 
	                          	     // even if we want to just configure it again to whatever we actually want. Then we need to wait again.
	S_LIDAR_PRECONF_CHECK_SPEED	= 4, // If the speed happenes to be configured to what we actually want, we can skip the next one:
	S_LIDAR_CONF_SPEED		= 5,
	S_LIDAR_WAIT_READY		= 6,
	S_LIDAR_CONF_SAMPLING		= 7,
	S_LIDAR_WAIT_START_ACK		= 8,
	S_LIDAR_RUNNING			= 9,
	S_LIDAR_RECONF			= 10,
	S_LIDAR_ERROR			= 11
} lidar_state_t;

extern lidar_state_t cur_lidar_state;

typedef enum
{
	LIDAR_NO_ERROR				= 0,
	LIDAR_ERR_RX_DMA_BUSY			= 1,
	LIDAR_ERR_TX_DMA_BUSY			= 2,
	LIDAR_ERR_SENSOR_ERRFLAGS 		= 3,
	LIDAR_ERR_CHKSUM_OR_ERRFLAGS 		= 4,
	LIDAR_ERR_UNEXPECTED_REPLY_MOTORSPEED 	= 5,
	LIDAR_ERR_UNEXPECTED_REPLY_SAMPLERATE 	= 6,
	LIDAR_ERR_DATASTART_ACK			= 7,
	LIDAR_ERR_STATE_WATCHDOG		= 8,
} lidar_error_t;

extern uint8_t lidar_error_flags;
extern lidar_error_t lidar_error_code;


#define LIVELIDAR_INVALID 1

typedef struct
{
	int status;
	int id;
	pos_t pos[90]; // Each four points share the same position.
	int16_t scan[360];
} live_lidar_scan_t;


void init_lidar();
void deinit_lidar();

void lidar_on(int fps, int smp);
void lidar_off();
void lidar_fsm();

void generate_lidar_ignore();
void copy_lidar_half1(int16_t* dst_start);
void copy_lidar_half2(int16_t* dst_start);
void copy_lidar_full(int16_t* dst_start);

void lidar_reset_flags();
void lidar_reset_complete_flag(); 
void lidar_reset_half_flag(); 
int lidar_is_complete();
int lidar_is_half();

void lidar_fsm();

void set_lidar_id(int id);

void lidar_mark_invalid();

extern point_t lidar_collision_avoidance[360];

extern volatile int lidar_near_filter_on;
extern volatile int lidar_midlier_filter_on;

#endif
