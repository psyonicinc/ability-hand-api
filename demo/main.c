#include "i2c-master-test.h"

/*Uncomment and compile whichever control mode you would like to test.*/
#define POS_CONTROL_MODE
//#define TAU_CONTROL_MODE
//#define VELOCITY_CONTROL_MODE

/*When enabled, prints the value of the pressure sensors on the index finger. */
#define PRINT_PRESSURE

float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}

void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	set_grip(GENERAL_OPEN_CMD,0xFF);	
	usleep(1000000);
	set_grip(CHUCK_OK_GRASP_CMD,0xFF);	
	usleep(1000000);
	set_grip(GENERAL_OPEN_CMD,0xFF);	
	usleep(1000000);	
	
	//set_mode(DISABLE_PRESSURE_FILTER);	//uncomment for RAW pressure
	//set_mode(DISABLE_TORQUE_VELOCITY_SAFETY);	//uncomment for UNSAFE torque and velocity control modes
	
	/*Setpoint generation start time*/
	struct timeval tv;
	float start_ts = current_time_sec(&tv);
	
	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	pres_union_fmt_i2c pres_fmt;

	/*Setup for demo motion*/
	float qd[NUM_CHANNELS] = {15.0f, 15.0f, 15.0f, 15.0f, 15.0f, -15.0f};
	float qd_amp[NUM_CHANNELS] = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, -50.0f};
	float qd_offset[NUM_CHANNELS] = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, -10.0f};
	
	while(1)
	{
		float t = current_time_sec(&tv) - start_ts;

		#ifdef POS_CONTROL_MODE
			int rc = send_recieve_floats(POS_CTL_MODE, &i2c_out, &i2c_in, &pres_fmt);
		#elif defined TAU_CONTROL_MODE
			int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &pres_fmt);
		#elif defined VELOCITY_CONTROL_MODE
			int rc = send_recieve_floats(VELOCITY_CTL_MODE, &i2c_out, &i2c_in, &pres_fmt);
		#endif
		
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		
		/*
		Pressure Indices:
		Index: 	0-3
		Middle: 4-7
		Ring: 	8-11
		Pinky: 	12-15
		Thumb: 	16-19
		
		Note that the The pressure range is NOT normalized (i.e. will range from 0-0xFFFF).
		*/			
		#ifdef PRINT_PRESSURE
			int ch;
			for(ch = 0; ch < 3; ch++)
				printf("ps[%d] = %f, ", ch,  (float)(pres_fmt.v[ch])/6553.5f );	//pressure will be 0-0xFFFF, floating point
			printf("ps[%d] = %f \r\n", ch, (float)(pres_fmt.v[ch])/6553.5f);
		#else	//Print the position
			int ch;
			for(ch = 0; ch < NUM_CHANNELS-1; ch++)
				printf("q[%d] = %f, ",ch,i2c_in.v[ch]);
			printf("q[%d] = %f\r\n",ch,i2c_in.v[ch]);
		#endif

		/*Generate a motion pattern*/
		float phase[NUM_CHANNELS] = {3, 2, 1, 0, 5, 4};
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			float sin_core = sin(t + phase[ch]*M_PI/12);
			sin_core = sin_core*sin_core;
			sin_core = sin_core*sin_core;
			sin_core = sin_core*sin_core;

			qd[ch] = qd_amp[ch]*(sin_core)+qd_offset[ch];
			
			#ifdef POS_CONTROL_MODE
				i2c_out.v[ch] = qd[ch];
			#elif defined TAU_CONTROL_MODE
				i2c_out.v[ch] = 2.0f*(qd[ch]-i2c_in.v[ch]);
			#elif defined VELOCITY_CONTROL_MODE
				i2c_out.v[ch] = 4.0f*(qd[ch]-i2c_in.v[ch]);
			#endif
		}
	}

}

