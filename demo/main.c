#include "i2c-master-test.h"

/*Uncomment and compile whichever control mode you would like to test.*/
#define POS_CONTROL_MODE
//#define TAU_CONTROL_MODE
//#define VELOCITY_CONTROL_MODE


//#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */

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
	return max;
}

void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);
//	set_grip(CHUCK_OK_GRASP_CMD,100);
//	usleep(2000000);
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
	
	int prev_phase = 0;
	int phase = 0;
		
	float q_stop[NUM_CHANNELS] = {0};
	float qd[NUM_CHANNELS] = {0};
	qd[THUMB_ROTATOR] = -80.f;

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
			int ch;
			const char * name[5] = {"index","middle","ring","pinky","thumb"};
			for(ch = 0; ch < 5; ch++)
			{
				int i_max = get_idx_of_max_pressure(ch, &pres_fmt);
				if(i_max >= 0)
					printf("%s: %.3f, ", name[ch], (float)pres_fmt.v[i_max]/6553.5f);
				else
					printf("%s: 0.000", name[ch]);
			}
			//	printf("ps[%d] = %f, ", ch,  (float)(pres_fmt.v[ch])/6553.5f );	//pressure will be 0-0xFFFF, floating point
			//printf("ps[%d] = %f\r\n", ch, (float)(pres_fmt.v[ch])/6553.5f);
		#else	//Print the position
			int ch;
			for(ch = 0; ch < NUM_CHANNELS-1; ch++)
				printf("q[%d] = %f, ",ch,i2c_in.v[ch]);
			printf("q[%d] = %f\r\n",ch,i2c_in.v[ch]);
		#endif
		printf("disabled status = ");
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			printf("%d", ((disabled_stat >> ch) & 1) );
		printf("\r\n");

		
		
		#ifdef POS_CONTROL_MODE
			
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = qd[ch];
			int rc = send_recieve_floats(POS_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		#elif defined TAU_CONTROL_MODE
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = 2.0f*(qd[ch] - i2c_in.v[ch]);
			int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		#elif defined VELOCITY_CONTROL_MODE
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = 3.0f*(qd[ch] - i2c_in.v[ch]);
			int rc = send_recieve_floats(VELOCITY_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		#endif
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
	}
}

