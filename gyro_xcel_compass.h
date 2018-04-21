/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

#ifndef _GYRO_XCEL_COMPASS_H
#define _GYRO_XCEL_COMPASS_H

#include <stdint.h>

#ifdef PCB1A

#define LSM303C

#define GYRO_I2C_ADDR 0x40

#define XCEL_I2C_ADDR 0x3A
#define COMPASS_I2C_ADDR 0x3C

#endif

#ifdef PCB1B

#define LSM303A

#define EXTRAGYRO

#define GYRO_I2C_ADDR 0x40
#define EXTRAGYRO_I2C_ADDR 0x42

#define XCEL_I2C_ADDR 0x32
#define COMPASS_I2C_ADDR 0x3C

#endif


#if !defined(LSM303C) && !defined(LSM303A)
#error LSM303C or LSM303A must be defined
#endif
#ifdef   __cplusplus
extern "C" {
#endif

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} gyro_data_t;

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} xcel_data_t;


typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} compass_data_t;

int init_gyro_xcel_compass(void);

#define GYRO_NEW_DATA 		((uint32_t)1)
#define XCEL_NEW_DATA 		((uint32_t)2)
#define COMPASS_NEW_DATA 	((uint32_t)4)
#define EXTRAGYRO_NEW_DATA 	((uint32_t)8)

uint32_t gyro_xcel_compass_fsm(void);

extern volatile gyro_data_t gyro_data[2];
extern volatile xcel_data_t xcel_data[2];
extern volatile compass_data_t compass_data[2];

extern volatile gyro_data_t *latest_gyro;
#ifdef EXTRAGYRO
extern volatile gyro_data_t *latest_extragyro;
#endif
extern volatile xcel_data_t *latest_xcel;
extern volatile compass_data_t *latest_compass;

extern volatile int gyro_timestep_plusses;
extern volatile int gyro_timestep_minuses;
extern volatile int xcel_timestep_plusses;
extern volatile int xcel_timestep_minuses;
#ifdef   __cplusplus
}
#endif

#endif
