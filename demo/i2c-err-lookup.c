#include "i2c-err-lookup.h"

#define ERR_LOOKUP_TABLE_SIZE 8

/*
Error code lookup table
*/
const char * err_code[ERR_LOOKUP_TABLE_SIZE] = {
	"no error",
	"error: master tx nack",
	"error: master rx nack",
	"error: I2C disconnected",
	"error: checksum mismatch",
	"error: checksum + tx nack",
	"error: checksum + rx nack",
	"error: checksum mismatch, I2C disconnected"
};

/*
Print the error code from the lookup table, guarding against 
bad indexes. If there is no error nothing prints.
*/
void print_hr_errcode(int rc)
{
	if(rc < ERR_LOOKUP_TABLE_SIZE && rc > 0)	//rc of 0 = no error
		printf("%s\r\n", err_code[rc]);
}
