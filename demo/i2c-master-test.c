#include "i2c-master-test.h"

static int file_i2c;
uint8_t i2c_tx_buf[I2C_TX_SIZE] = {0};
uint8_t i2c_rx_buf[I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE] = {0};

/*
Open the I2C port with default settings.
*/
int open_i2c(uint8_t addr)
{
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
		return -1;
	
	/*
		TODO: Initialize I2C with a higher speed? (Default 100kHZ, hand will support higher speeds)
	*/

	// ------ Configure the I2C bus -------
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
		return -2;
	
	return 0;
}

/*
Generic function which blindly sends out a hand grip index. 
NOTE: Recommended to start here!

INPUTS:
-grip_idx: 
	the index of the grip in question
-speed: 
	0 -> hand stops altogether
	1-254 -> hand speed maps linearly, from .5 'grips/sec' to 3.33333333 'grips/sec' 
	255 -> hand speed set to 5 'grips/sec' 
	
	Note: intuition for grips/sec is the grip 'period', i.e. 3.3333 'grips/sec' -> .3s to complete the grip
*/
int set_grip(grasp_cmd grip_idx, uint8_t speed)
{
	const int len = 3;
	uint8_t i2c_buf[len];
	i2c_buf[0] = GRIP_CTL_MODE;
    i2c_buf[1] = (uint8_t)grip_idx;
	i2c_buf[2] = speed;
	if (write(file_i2c, i2c_buf, len) != len)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
		return -1;
	return 0;
}

/*
Function which enables various control modes. Is used also to enable/disable certain features, such as 
pressure filtering and motor safety.
*/
int set_mode(uint8_t mode)
{
	const int len = 3;                     //<<< Number of bytes to write
	uint8_t i2c_tx_buf[len];
	i2c_tx_buf[0] = mode;
	i2c_tx_buf[1] = 0x00;
	i2c_tx_buf[2] = 0x00;
	if (write(file_i2c, i2c_tx_buf, len) != len)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
		return -1;
	return 0;
}


/*
API, low level control mode. 
INPUTS: 
	mode: The control mode (position, torque, or velocity)
	out:  Floating point numbers (6) to send to the hand. Interpreted differently for each control mode.
			-In Position control mode, out.v is a list of angles (in degrees) that the finger will move to.
			-In Velocity control mode, out.v is a list of velocities in degrees/second
			-In Torque control mode, out.v is a list of unitless torques. Useful range is -80 to 80
OUTPUTS:
	in: 			Always corresponds to finger position (zero-referenced from the stall point on startup), in degrees.
	pres_fmt: 		Contains the pressure sensor data ranging from 0-0xFFFF
	disabled_stat: 	Pass by reference word that indicates the driver disabled status (safety feature)
		Bit:[ 		0			1				2			3					4						5			]
			[	index 0/1	middle 0/1		ring 0/1	pinky 0/1		thumb flexor 0/1		thumb rotator 0/1	]

Returns a nonzero code to indicate I2C error.
*/
int send_recieve_floats(uint8_t mode, float_format_i2c * out, float_format_i2c * in, uint8_t * disabled_stat, pres_union_fmt_i2c * pres_fmt)
{
	int ret = 0;
	
	i2c_tx_buf[0] = mode;
	for(int i = 0; i < I2C_Q_RX_SIZE; i++)
		i2c_tx_buf[i+1] = out->d[i];
	
	if (write(file_i2c, i2c_tx_buf, I2C_TX_SIZE) != I2C_TX_SIZE)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
		ret |= 1;
	if(read(file_i2c, i2c_rx_buf, I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE) != I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE)
		ret |= (1 << 1);
	else
	{
		for(int i = 0; i < I2C_Q_RX_SIZE; i++)
			in->d[i] = i2c_rx_buf[i];
		for(int i = 24; i < 64; i++)
			pres_fmt->d[i-24] = i2c_rx_buf[i];
	}
	*disabled_stat = i2c_rx_buf[64];
	
	return ret;
}
int send_enable_word(uint8_t enable_command)
{
	int ret = 0;
	uint8_t confirm = 0;
	int attempts = 0;
	while(confirm == 0)
	{
		for(int i = 0; i < I2C_TX_SIZE; i++)
			i2c_tx_buf[i]=0;	
		i2c_tx_buf[0] = 0xEB;
		i2c_tx_buf[1] = enable_command;
		if(write(file_i2c, i2c_tx_buf, I2C_TX_SIZE) != I2C_TX_SIZE)
			ret|=1;
		if(read(file_i2c, i2c_rx_buf, I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE) != I2C_Q_RX_SIZE+I2C_PS_TX_SIZE+I2C_SAFETY_STAT_SIZE)
			ret |= (1 << 1);
		if(i2c_rx_buf[64] == enable_command)
			confirm = 1;
		
		attempts++;
		if(attempts > 300)	//Kludged timeout
		{
			confirm = 1;	//break out of loop with an error message
			ret |= (1 << 2);
		}
	}
	
	return ret;
}
