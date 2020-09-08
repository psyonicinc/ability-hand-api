#ifndef I2C_MASTER_TEST_H
#define I2C_MASTER_TEST_H

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

/*Various size defines for i2c communication*/
#define NUM_CHANNELS 6
#define I2C_PS_TX_SIZE 40
#define I2C_TX_SIZE 25
#define I2C_Q_RX_SIZE 24
#define I2C_SAFETY_STAT_SIZE 1

/*Set mode flags*/
#define DISABLE_PRESSURE_FILTER				0xC0		
#define ENABLE_PRESSURE_FILTER				0xC1	//enabled by default
#define DISABLE_TORQUE_VELOCITY_SAFETY		0xB0
#define ENABLE_TORQUE_VELOCITY_SAFETY		0xB1	//enabled by default
#define POS_CTL_MODE						0xAD
#define TORQUE_CTL_MODE						0xAB
#define VELOCITY_CTL_MODE					0xAC
#define GRIP_CTL_MODE 						0x1D

/*Motor indices*/
#define INDEX 			0
#define MIDDLE 			1
#define RING 			2
#define PINKY 			3
#define THUMB_FLEXOR 	4
#define THUMB_ROTATOR 	5

/*Union used for easy data stream type conversion (for motors)*/
typedef union
{
	uint8_t d[I2C_Q_RX_SIZE];
	float v[NUM_CHANNELS];
}float_format_i2c;

/*Union used for pressure formatting and type conversion*/
typedef union
{
	uint8_t d[40];
	uint16_t v[20];
}pres_union_fmt_i2c;

typedef enum {
	GENERAL_OPEN_CMD,
	POWER_GRASP_CMD,
	KEY_GRASP_CMD,
	PINCH_GRASP_CMD,
	CHUCK_OK_GRASP_CMD,
	SIGN_OF_THE_HORNS_GRASP,
	OPEN_THUMB_DOWN_CMD,
	OPEN_THUMB_UP_CMD,
	MODE_SWITCH_CLOSE_CMD,
	POINT_GRASP_CMD,
	RUDE_POINT_CMD,
	MODE_SWITCH_OPEN_CMD,
	RELAX_CMD,
	THUMB_DOWN_OPEN_GRASP_CMD,
	OPEN_KEY_CMD,
	CHUCK_SINGLE_PRIO_CMD,//
	KEY_SINGLE_PRIO_CMD,
	HANDSHAKE_CMD,
	ROTATOR_UP_CMD,
	ROTATOR_DOWN_CMD,
	ROTATOR_STOP_CMD,
	OPEN_HANDSHAKE_CMD,
	KEY_FINGERS_OPEN_CMD,	//key with finger motion on open
	OPEN_FOR_POWER_CMD,
	TRIGGER_CMD,
	OPEN_TRIGGER_CMD,
	OPEN_POINT_CMD,
	CHUCK_GRASP_CMD,
	OPEN_CHUCK_OK_GRASP_CMD
}grasp_cmd;

int open_i2c(uint8_t addr);
int set_grip(grasp_cmd grip_idx, uint8_t speed);
int set_mode(uint8_t mode);
int send_recieve_floats(uint8_t mode, float_format_i2c * out, float_format_i2c * in, uint8_t * disabled_stat, pres_union_fmt_i2c * pres_fmt);
int send_enable_word(uint8_t enable_command);
//int send_recieve_floats(uint8_t mode, float_format_i2c * out, float_format_i2c * in, pres_union_fmt_i2c * pres_fmt);


#endif
