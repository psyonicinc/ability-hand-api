#include "i2c-master-test.h"

float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}

/*INPUT 0-4
0-index
1-middle
2-ring
3-pinky
4-thumb
*/
int get_idx_of_max_pressure(int ch, pres_union_fmt_i2c * pres_fmt)
{
	if(ch > 4)
		ch = 4;
	else if (ch < 0)
		ch = 0;
	int lowidx = ch*4;
	int hidx = lowidx+4;
	uint16_t max = 0;
	int idx_of_max = -1;
	for(int i = lowidx; i < hidx; i++)
	{
		uint16_t v = pres_fmt->v[ch];
		if(max < v)
		{
			idx_of_max = i;
			max = v;
		}		
	}
	return idx_of_max;
}

void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);

	//set_mode(DISABLE_PRESSURE_FILTER);	//uncomment for RAW pressure
	//set_mode(DISABLE_TORQUE_VELOCITY_SAFETY);	//uncomment for UNSAFE torque and velocity control modes

	/*Setpoint generation start time*/
	struct timeval tv;

	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	pres_union_fmt_i2c pres_fmt;

	/*Setup for demo motion*/
	uint8_t disabled_stat = 0;

	float start_ts = current_time_sec(&tv);
	float max_of_max[5] = {0};
	while(1)
	{
		float t = fmod(current_time_sec(&tv) - start_ts, 10);

		/*
		Pressure Indices:
		Index: 	0-3
		Middle: 4-7
		Ring: 	8-11
		Pinky: 	12-15
		Thumb: 	16-19

		Note that the The pressure range is NOT normalized (i.e. will range from 0-0xFFFF).
		*/
		
		for(int finger = 0; finger < 5; finger++)
		{
			int i_max = get_idx_of_max_pressure(finger, &pres_fmt);
			if(i_max >= 0)
			{
				float v = (float)pres_fmt.v[i_max]/6553.5f;
				if(max_of_max[finger] < v)
					max_of_max[finger] = v;
			}
		}	
		const char * name[5] = {"index","middle","ring","pinky","thumb"};
		int finger = 0;
		for(finger = 0; finger < 4; finger++)
			printf("%s: %.3f, ", name[finger], max_of_max[finger]);
		printf("%s: %.3f\r\n", name[finger], max_of_max[finger]);

			
		for(int ch =0; ch < NUM_CHANNELS; ch++)
			i2c_out.v[ch] = 0.f;
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);	//no motor motion, just want the pressure sensor data
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
	}
}

