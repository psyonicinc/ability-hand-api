#include "i2c-master-test.h"

/*Uncomment and compile whichever control mode you would like to test.*/
//const uint8_t gl_ctl_mode = TORQUE_CTL_MODE;
const uint8_t gl_ctl_mode = POS_CTL_MODE;
//const uint8_t gl_ctl_mode = VELOCITY_CTL_MODE;

//#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */
//#define PRINT_POSITION	/*When enabled, prints the finger position in degrees/*

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
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);
	set_grip(CHUCK_OK_GRASP_CMD,100);
	usleep(2000000);
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);

	//set_mode(DISABLE_PRESSURE_FILTER);	//uncomment for RAW pressure
	//set_mode(DISABLE_TORQUE_VELOCITY_SAFETY);	//uncomment for UNSAFE torque and velocity control modes

	/*Setpoint generation start time*/
	struct timeval tv;

	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	
	/*Setup for demo motion*/
	uint8_t disabled_stat = 0;
	
	int prev_phase = 0;
	int phase = 0;
		
	float q_stop[NUM_CHANNELS] = {0};
	float qd[NUM_CHANNELS] = {0};
	qd[THUMB_ROTATOR] = -80.f;

	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	set_mode(TORQUE_CTL_MODE);
	usleep(10000);
	for(float ts = current_time_sec(&tv) + .5; current_time_sec(&tv) < ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);	//initialize position
	set_mode(gl_ctl_mode);
	usleep(10000);
	
	float start_ts = current_time_sec(&tv);
	while(1)
	{
		float t = fmod(current_time_sec(&tv) - start_ts, 10);
		if(t >= 1 && t < 2)
		{
			phase = 1;
			float t_off = t-1.f;
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = t_off*50.f + 20.f;	//travel to 70 degrees (close) from 20 degrees (open)
			qd[THUMB_FLEXOR] = t_off*30.f+10.f; //travel to 40 degrees (close) from 10 degrees (open)
			
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				q_stop[ch] = i2c_in.v[ch];	//record so when phase 1 is complete you know where the hand stopped
		}
		else if(t >= 6 && t < 7)
		{
			phase = 2;
			float t_off = (t-6.f);
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = t_off*(20.f-q_stop[ch]) + q_stop[ch];	//travel to 20 from where you currently are
			qd[THUMB_FLEXOR] = t_off*(10.f-q_stop[THUMB_FLEXOR])+q_stop[THUMB_FLEXOR];	//travel to 10 from where you currently are
		}
		else if(t > 7)
		{
			phase = 3;
			for(int ch = 0; ch <= PINKY; ch++)
				qd[ch] = 20.f;			//enforce start position
			qd[THUMB_FLEXOR] = 10.f;		//for both sets of fingers
		}
		else
			phase = -1;

		if(prev_phase != phase && prev_phase == -1)
		{
			//enable_cmd = 0x3f;
			send_enable_word(0x3F);		//should call this only once for optimum behavior
			printf(" enabling...\r\n");
		}
		prev_phase = phase;
		
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
			int pidx = 0;
			for(pidx = 0; pidx < 3; pidx++)
				printf("%.3f, ",(float)pres_fmt.v[pidx]/6553.5f);
			printf("%.3f\r\n",(float)pres_fmt.v[pidx]/6553.5f);
		#elif defined PRINT_POSITION	//Print the position
			int ch;
			for(ch = 0; ch < NUM_CHANNELS-1; ch++)
				printf("q[%d] = %f, ",ch,i2c_in.v[ch]);
			printf("q[%d] = %f\r\n",ch,i2c_in.v[ch]);
		#else
			const char * name[NUM_CHANNELS] = {"index","middle","ring","pinky","thumb flexor", "thumb rotator"};
			const char * yn[2] = {"on ","off"};
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				printf("%s: %s ", name[ch], yn[((disabled_stat >> ch) & 1)] );
			printf("\r\n");			
		#endif
		

		
		int rc = 0;
		switch(gl_ctl_mode)
		{
			case POS_CTL_MODE:
			{
				for(int ch = 0; ch < NUM_CHANNELS; ch++)
					i2c_out.v[ch] = qd[ch];
				rc = send_recieve_floats(POS_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
				break;
			}
			case TORQUE_CTL_MODE:
			{
				for(int ch = 0; ch < NUM_CHANNELS; ch++)
					i2c_out.v[ch] = 2.0f*(qd[ch] - i2c_in.v[ch]);
				int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
				break;
			}
			case VELOCITY_CTL_MODE:
			{
				for(int ch = 0; ch < NUM_CHANNELS; ch++)
					i2c_out.v[ch] = 3.0f*(qd[ch] - i2c_in.v[ch]);
				int rc = send_recieve_floats(VELOCITY_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
				break;
			}
			default:
				rc = 11;	//random value to show ctl mode is not set properly
			break;
		};
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
	}
}
