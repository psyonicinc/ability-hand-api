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

#define I2C_TX_SIZE 26	//0-24 = pos bytes. 25 = checksum

#define I2C_Q_RX_SIZE 24	//first set of bytes (0-23)
#define I2C_PS_TX_SIZE 45	//next set of bytes (24-63)
#define I2C_SAFETY_STAT_SIZE 1	//byte 69
#define I2C_CHECKSUM_SIZE 	1	//byte 70

#define I2C_RX_BUF_SIZE I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE+I2C_CHECKSUM_SIZE

/*Set mode flags*/
#define DISABLE_PRESSURE_FILTER				0xC0		
#define ENABLE_PRESSURE_FILTER				0xC1	//enabled by default
#define DISABLE_TORQUE_VELOCITY_SAFETY		0xB0
#define ENABLE_TORQUE_VELOCITY_SAFETY		0xB1	//enabled by default
#define POS_CTL_MODE						0xAD
#define TORQUE_CTL_MODE						0xAB
#define VELOCITY_CTL_MODE					0xAC
#define GRIP_CTL_MODE 						0x1D
#define READ_ONLY_MODE						0x31
/*Motor indices*/
#define INDEX 			0
#define MIDDLE 			1
#define RING 			2
#define PINKY 			3
#define THUMB_FLEXOR 	4
#define THUMB_ROTATOR 	5

#define NUM_FSR_PER_FINGER 6

/*Union used for easy data stream type conversion (for motors)*/
typedef union
{
	uint8_t d[I2C_Q_RX_SIZE];
	float v[NUM_CHANNELS];
}float_format_i2c;

/**/
typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

/*Union used for pressure formatting and type conversion*/
typedef struct
{
	uint16_t v[NUM_FSR_PER_FINGER];
}pres_fmt_i2c;

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
int api_frame_fmt_1(uint8_t mode, float_format_i2c * out, float * fpos, uint8_t * disabled_stat, pres_fmt_i2c * pres_fmt);
int api_frame_fmt_2(uint8_t mode, float_format_i2c * out, float fpos[NUM_CHANNELS], float iq[NUM_CHANNELS], uint8_t * disabled_stat, pres_fmt_i2c * pres_fmt);


#endif
