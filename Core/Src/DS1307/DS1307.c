#include "DS1307.h"

uint8_t DS1307_DecodeBCD(uint8_t bin) {				// bcd to dec
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}
uint8_t DS1307_EncodeBCD(uint8_t dec) {				// dec to bcd
	return (dec % 10 + ((dec / 10) << 4));
}

void DS1307_gettime(I2C_HandleTypeDef *hi2c, DS1307_param_t *ds1307) {
	uint16_t cen;
	ds1307->sec = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_SECOND) & 0x7f);
	ds1307->min = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_MINUTE));
	ds1307->hour = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_HOUR) & 0x3f);
	ds1307->dow = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_DOW));
	ds1307->date = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_DATE));
	ds1307->month = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_MONTH));
	cen = DS1307_GetRegByte(hi2c, DS1307_REG_CENT) * 100;
	ds1307->year = DS1307_DecodeBCD(DS1307_GetRegByte(hi2c, DS1307_REG_YEAR)) + cen;
}
void DS1307_SetRegByte(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t val) {
	uint8_t bytes[2] = { regAddr, val };
	HAL_I2C_Master_Transmit(hi2c, DS1307_I2C_ADDR << 1, bytes, 2,
	DS1307_TIMEOUT);
}
uint8_t DS1307_GetClockHalt(I2C_HandleTypeDef *hi2c) {
	return (DS1307_GetRegByte(hi2c, DS1307_REG_SECOND) & 0x80) >> 7;
}
void DS1307_settime(I2C_HandleTypeDef *hi2c, uint8_t sec, uint8_t min, uint8_t hour_24mode,
		uint8_t dayOfWeek, uint8_t date, uint8_t month, uint16_t year) {
	DS1307_SetRegByte(hi2c, DS1307_REG_SECOND,
			DS1307_EncodeBCD(sec | DS1307_GetClockHalt(hi2c)));
	DS1307_SetRegByte(hi2c, DS1307_REG_MINUTE, DS1307_EncodeBCD(min));
	DS1307_SetRegByte(hi2c, DS1307_REG_HOUR, DS1307_EncodeBCD(hour_24mode & 0x3f)); //hour_24mode Hour in 24h format, 0 to 23.
	DS1307_SetRegByte(hi2c, DS1307_REG_DOW, DS1307_EncodeBCD(dayOfWeek)); //dayOfWeek Days since last Sunday, 0 to 6.
	DS1307_SetRegByte(hi2c, DS1307_REG_DATE, DS1307_EncodeBCD(date)); //date Day of month, 1 to 31.
	DS1307_SetRegByte(hi2c, DS1307_REG_MONTH, DS1307_EncodeBCD(month)); //month Month, 1 to 12.
	DS1307_SetRegByte(hi2c, DS1307_REG_CENT, year / 100);
	DS1307_SetRegByte(hi2c, DS1307_REG_YEAR, DS1307_EncodeBCD(year % 100)); //2000 to 2099.
}
uint8_t DS1307_GetRegByte(I2C_HandleTypeDef *hi2c, uint8_t regAddr) {
	uint8_t val;
	HAL_I2C_Master_Transmit(hi2c, DS1307_I2C_ADDR << 1, &regAddr, 1,
	DS1307_TIMEOUT);
	HAL_I2C_Master_Receive(hi2c, DS1307_I2C_ADDR << 1, &val, 1,
	DS1307_TIMEOUT);
	return val;
}
void DS1307_SetClockHalt(I2C_HandleTypeDef *hi2c, uint8_t halt) {
	uint8_t ch = (halt ? 1 << 7 : 0);
	DS1307_SetRegByte(hi2c, DS1307_REG_SECOND,
			ch | (DS1307_GetRegByte(hi2c, DS1307_REG_SECOND) & 0x7f));
}
/**
 * @brief Sets UTC offset.
 * @note  UTC offset is not updated automatically.
 * @param hr UTC hour offset, -12 to 12.
 * @param min UTC minute offset, 0 to 59.
 */
void DS1307_SetTimeZone(I2C_HandleTypeDef *hi2c, int8_t hr, uint8_t min) {
	DS1307_SetRegByte(hi2c, DS1307_REG_UTC_HR, hr);
	DS1307_SetRegByte(hi2c, DS1307_REG_UTC_MIN, min);
}
void DS1307_config(I2C_HandleTypeDef *hi2c) {
	DS1307_SetClockHalt(hi2c, 0);
	DS1307_SetTimeZone(hi2c, +8, 00);
}

