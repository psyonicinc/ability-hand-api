/*
  Example program to send a 'wiggling fingers' command sequence to an Ability Hand
  using UART.

  Note: Please check that your Arduino is able to accurately service the requested baud rate, AND 
  that it's UART is 3.3V level!!!!

  For instance, an Arduino is 5V level and can't support the default baud rate (460800) due to 
  limitations of the clock and division used to achieve UART baud.
*/
#include <stdint.h>
#include <math.h>

#define NUM_CHANNELS 6
#define API_TX_SIZE	 15

typedef union api_i16_t
{
	int16_t i16[NUM_CHANNELS];
	uint8_t u8[NUM_CHANNELS*sizeof(int16_t)];
}api_i16_t;


/*Helper function to get the signed 8bit checksum*/
uint8_t get_checksum(uint8_t * arr, int size)
{
	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

/*Takes 6x floating point inputs for hand position arguments in DEGREES, and creates an
API frame to send out*/
void format_packet(float fpos_in[NUM_CHANNELS], uint8_t tx_buf[API_TX_SIZE])
{
	tx_buf[0] = 0x50; //hand slave address (use default)	
	tx_buf[1] = 0x12;
	api_i16_t pld;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		pld.i16[ch] = (int16_t)((fpos_in[ch] * 32767.0f) / 150.0f);
	}
	for(int i = 0; i < NUM_CHANNELS * sizeof(int16_t); i++)
	{
		tx_buf[i+2] = pld.u8[i];
	}
	tx_buf[API_TX_SIZE-1] = get_checksum((uint8_t*)tx_buf, API_TX_SIZE-1);  //full checksum
}

void setup() {
	/*
	Be sure to check your Arduino can support this baud! Some will round
	it off due to limitations in how the clock can be divided
	*/
	Serial.begin(460800);	
}

void loop()
{
	float fpos[NUM_CHANNELS] = {15.f,15.f,15.f,15.f,15.f,-15.f};
	uint8_t tx_buf[API_TX_SIZE] = {0};
	while(1)
	{
		float t = ((float)millis())*.001f;
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			fpos[ch] = (0.5f*cos(t + (float)ch)+0.5f)*30.f + 15.f;
		}
		fpos[5] = -fpos[5];

		format_packet(fpos, tx_buf);
		Serial.write(tx_buf, 15);
		delay(5);
	}
}