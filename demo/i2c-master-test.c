#include "i2c-master-test.h"
#include "m_mcpy.h"

static int file_i2c;
uint8_t i2c_tx_buf[I2C_TX_SIZE] = {0};
uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] = {0};
const float fixed_fpos_conv_factor = (150.f/ ((float)(0x7FFF)) );	//should be eval at compile time
const float fixed_iq_to_amps_div = 4603.613636f;

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
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint8_t get_checksum(uint8_t * arr, int size)
{

	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
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
	for(int i = 1; i < I2C_TX_SIZE-1; i++)
		i2c_tx_buf[i] = 0;
	i2c_tx_buf[0] = mode;
	i2c_tx_buf[I2C_TX_SIZE-1] = get_checksum(i2c_tx_buf, I2C_TX_SIZE-1);
	if (write(file_i2c, i2c_tx_buf, I2C_TX_SIZE) != I2C_TX_SIZE)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
		return -1;
	return 0;
}

/*
* Load packed 12 bit values located in an 8bit array into
* an unpacked (zero padded) 16 bit array. FSR utility function
*/
void unpack_8bit_into_12bit(uint8_t* arr, uint16_t* vals, int valsize)
{
	for(int i = 0; i < valsize; i++)
		vals[i] = 0;	//clear the buffer before loading it with |=
    for (int bidx = valsize * 12 - 4; bidx >= 0; bidx -= 4)
    {
        int validx = bidx / 12;
        int arridx = bidx / 8;
        int shift_val = (bidx % 8);
        vals[validx] |= ((arr[arridx] >> shift_val) & 0x0F) << (bidx % 12);
    }
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
int api_frame_fmt_1(uint8_t mode, float_format_i2c * out, float * fpos, uint8_t * disabled_stat, pres_fmt_i2c * pres_fmt)
{
	int ret = 0;
	uint8_t checksum = 0;
	
	if(mode != READ_ONLY_MODE)
	{
		i2c_tx_buf[0] = mode;
		for(int i = 0; i < I2C_Q_RX_SIZE; i++)
			i2c_tx_buf[i+1] = out->d[i];
		
		i2c_tx_buf[I2C_TX_SIZE-1] = get_checksum(i2c_tx_buf, I2C_TX_SIZE-1); //deliberate break of checksum for testing purposes
		
		if (write(file_i2c, i2c_tx_buf, I2C_TX_SIZE) != I2C_TX_SIZE)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
			ret |= 1;
	}
	if(read(file_i2c, i2c_rx_buf, I2C_RX_BUF_SIZE) != I2C_RX_BUF_SIZE)
		ret |= (1 << 1);
	else
	{
		float_format_i2c in;
		for(int i = 0; i < I2C_Q_RX_SIZE; i++)
			in.d[i] = i2c_rx_buf[i];
		for(int i = 0; i < NUM_CHANNELS; i++)
			fpos[[i] = in.v[i];
		//for(int i = I2C_Q_RX_SIZE; i < I2C_Q_RX_SIZE+I2C_PS_TX_SIZE; i++)
		//	pres_fmt->d[i-I2C_Q_RX_SIZE] = i2c_rx_buf[i];
		for(int sensor = 0; sensor < 5; sensor++)
		{
			int start_bidx = I2C_Q_RX_SIZE + 9*sensor;
			unpack_8bit_into_12bit(&i2c_rx_buf[start_bidx], pres_fmt[sensor].v, NUM_FSR_PER_FINGER);
		}
	}
	
	*disabled_stat = i2c_rx_buf[I2C_RX_BUF_SIZE-2];
	
	checksum = get_checksum(i2c_rx_buf, I2C_RX_BUF_SIZE-1);
	
	if(checksum != i2c_rx_buf[I2C_RX_BUF_SIZE-1])
		ret |= (1 << 2);
	
	return ret;
}


/*
API, low level control mode 2. 
INPUTS: 
	mode: The control mode (position, torque, or velocity)
	out:  Floating point numbers (6) to send to the hand. Interpreted differently for each control mode.
			-In Position control mode, out.v is a list of angles (in degrees) that the finger will move to.
			-In Velocity control mode, out.v is a list of velocities in degrees/second
			-In Torque control mode, out.v is a list of unitless torques. Useful range is -80 to 80
OUTPUTS:
	
	pres_fmt: 		Contains the pressure sensor data ranging from 0-0xFFFF
	disabled_stat: 	Pass by reference word that indicates the driver disabled status (safety feature)
		Bit:[ 		0			1				2			3					4						5			]
			[	index 0/1	middle 0/1		ring 0/1	pinky 0/1		thumb flexor 0/1		thumb rotator 0/1	]

Returns a nonzero code to indicate I2C error.
*/
int api_frame_fmt_2(uint8_t mode, float_format_i2c * out, float fpos[NUM_CHANNELS], float iq[NUM_CHANNELS], uint8_t * disabled_stat, pres_fmt_i2c * pres_fmt)
{
	int ret = 0;
	uint8_t checksum = 0;
	
	if(mode != READ_ONLY_MODE)
	{
		i2c_tx_buf[0] = mode;
		for(int i = 0; i < I2C_Q_RX_SIZE; i++)
			i2c_tx_buf[i+1] = out->d[i];
		
		i2c_tx_buf[I2C_TX_SIZE-1] = get_checksum(i2c_tx_buf, I2C_TX_SIZE-1); //deliberate break of checksum for testing purposes
		
		if (write(file_i2c, i2c_tx_buf, I2C_TX_SIZE) != I2C_TX_SIZE)          //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
			ret |= 1;
	}
	
	if(read(file_i2c, i2c_rx_buf, I2C_RX_BUF_SIZE) != I2C_RX_BUF_SIZE)
		ret |= (1 << 1);
	else
	{
		/*Load the i2c byte array into an array of type punned words*/
		u32_fmt_t api_motor_fmt_arr[I2C_Q_RX_SIZE/sizeof(u32_fmt_t)];	//6 total words. each word is a dual int16 containing finger position and current
		m_mcpy((void*)&api_motor_fmt_arr, (void*)i2c_rx_buf, I2C_Q_RX_SIZE);
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			fpos[ch] = ((float)api_motor_fmt_arr[ch].i16[0])*fixed_fpos_conv_factor;
			iq[ch] = (float)api_motor_fmt_arr[ch].i16[1]/fixed_iq_to_amps_div;
		}
		for(int sensor = 0; sensor < 5; sensor++)
		{
			int start_bidx = I2C_Q_RX_SIZE + 9*sensor;
			unpack_8bit_into_12bit(&i2c_rx_buf[start_bidx], pres_fmt[sensor].v, NUM_FSR_PER_FINGER);
		}
	}
	
	*disabled_stat = i2c_rx_buf[I2C_RX_BUF_SIZE-2];
	
	checksum = get_checksum(i2c_rx_buf, I2C_RX_BUF_SIZE-1);
	
	if(checksum != i2c_rx_buf[I2C_RX_BUF_SIZE-1])
		ret |= (1 << 2);
	
	return ret;
}
