#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

#include "stm32f4xx_hal.h"

float calibration(uint16_t signal);
float current(float scale, uint16_t signal);
uint16_t getSignal();
void startADC();

#endif