#include "stdint.h"
//#include "stm32f1xx_hal.h"
#include "stm32f4xx_hal.h"

#define DS1307_I2C_ADDR    	 	0x68
#define DS1307_REG_SECOND    	0x00
#define DS1307_REG_MINUTE    	0x01
#define DS1307_REG_HOUR      	0x02
#define DS1307_REG_DOW       	0x03
#define DS1307_REG_DATE      	0x04
#define DS1307_REG_MONTH     	0x05
#define DS1307_REG_YEAR      	0x06
#define DS1307_REG_CONTROL   	0x07
#define DS1307_REG_UTC_HR    	0x08
#define DS1307_REG_UTC_MIN   	0x09
#define DS1307_REG_CENT      	0x10
#define DS1307_TIMEOUT       	1000

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dow;
	uint8_t date;
	uint8_t month;
	uint16_t year;
} DS1307_param_t;

uint8_t DS1307_DecodeBCD(uint8_t bin);
uint8_t DS1307_EncodeBCD(uint8_t dec);
void DS1307_SetClockHalt(I2C_HandleTypeDef *hi2c, uint8_t halt);
void DS1307_SetRegByte(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t val);
void DS1307_SetTimeZone(I2C_HandleTypeDef *hi2c, int8_t hr, uint8_t min);
uint8_t DS1307_GetClockHalt(I2C_HandleTypeDef *hi2c);
uint8_t DS1307_GetRegByte(I2C_HandleTypeDef *hi2c, uint8_t regAddr);
void DS1307_config(I2C_HandleTypeDef *hi2c);
void DS1307_gettime(I2C_HandleTypeDef *hi2c, DS1307_param_t *ds1307);
void DS1307_settime(I2C_HandleTypeDef *hi2c, uint8_t sec, uint8_t min, uint8_t hour_24mode,
		uint8_t dayOfWeek, uint8_t date, uint8_t month, uint16_t year);
