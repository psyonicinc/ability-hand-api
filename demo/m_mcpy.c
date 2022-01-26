/*
 * m_mcpy.c
 *
 *  Created on: Dec 1, 2021
 *      Author: Ocanath
 */
#include "m_mcpy.h"
 /*
  *
  * */
void m_mcpy(void* dest, void* src, int num_bytes)
{
	uint8_t* d = (uint8_t*)dest;
	uint8_t* s = (uint8_t*)src;
	for (int i = 0; i < num_bytes; i++)
	{
		d[i] = s[i];
	}
}
