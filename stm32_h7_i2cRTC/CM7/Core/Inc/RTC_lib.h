/*
 * RTC_lib.h
 *
 *  Created on: Oct 7, 2022
 *      Author: iandu
 */

//https://www.mouser.com/datasheet/2/256/DS3231-1513891.pdf

#ifndef INC_RTC_LIB_H_
#define INC_RTC_LIB_H_

#include <stdio.h>
#include "main.h"




int bcdToDec(uint8_t val);

uint8_t decToBcd(int val);

void SetTime(uint16_t RTC_ADDR, uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint8_t year);

void getCurrentTime(uint16_t RTC_ADDR, uint8_t buff[], uint16_t Size);

void printCurrentTime(uint8_t buff[], uint16_t Size);
#endif /* INC_RTC_LIB_H_ */


