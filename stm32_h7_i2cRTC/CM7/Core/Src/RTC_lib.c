/*
 * RTC_lib.c
 *
 *  Created on: Oct 7, 2022
 *      Author: iandu
 */

#include "RTC_lib.h"

extern I2C_HandleTypeDef hi2c4;

/**
 * @brief  Returns BCD Value of a Decimal Number
 * @param  val integer
 * @retval None
 */
uint8_t decToBcd(int val) {
	return (uint8_t) ((val / 10 * 16) + (val % 10));
}

/**
 * @brief  Convert binary coded decimal to normal decimal numbers
 * @param  None
 * @retval None
 */
int bcdToDec(uint8_t val) {
	return (int) ((val / 16 * 10) + (val % 16));
}

/**
  * @brief  Sets the time of the RTC given specific parameters
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  sec current seconds 0-59
  * @param  min current minutes 0-59
  * @param  hour current hours 0-24
  * @param  day current day 1-30
  * @param  month current month 1-12
  * @param  year current year 0-99 with offset 2000
  */
void SetTime(uint16_t RTC_ADDR, uint8_t sec, uint8_t min, uint8_t hour,
		uint8_t day, uint8_t month, uint8_t year) {
	uint8_t buff[7];
	buff[0] = decToBcd(sec);
	buff[1] = decToBcd(min);
	buff[2] = decToBcd(hour);
	buff[4] = decToBcd(day);
	buff[5] = decToBcd(month);
	buff[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c4, RTC_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &buff,
			sizeof(buff), HAL_MAX_DELAY);

}

// Don't Use - No jala yet
void getCurrentTime(uint16_t RTC_ADDR, uint8_t buff[], uint16_t Size) {
	HAL_I2C_Mem_Read(&hi2c4, RTC_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &buff,
			sizeof(buff), HAL_MAX_DELAY);
}

/**
  * @brief  Prints formatted time given a buffer
  * @param  Buffer that contains bcd time values from seconds to year
  * @param  Size size  of the buffer
  */
void printCurrentTime(uint8_t buff[], uint16_t Size){
	if(Size!=7) return;
	int sec = bcdToDec(buff[0]);
	int min = bcdToDec(buff[1]);
	int hour = bcdToDec(buff[2]);
	int day = bcdToDec(buff[4]);
	int month = bcdToDec(buff[5]);
	int year = 2000+bcdToDec(buff[6]);
	printf("Elapsed time: %02d-%02d-%02d %02d:%02d:%02d \n\r", month, day, year, hour, min, sec);

}


