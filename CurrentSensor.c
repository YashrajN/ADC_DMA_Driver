#include"CurrentSensor.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

const float KNOWN_CURR = 10;


float calibration(uint16_t signal){
	return KNOWN_CURR/signal;
}

float current(float scale, uint16_t signal){
	float curr = scale*signal;
	return curr;
}

uint16_t getSignal(){
	uint16_t signal;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&signal, sizeof(signal));
	return signal;
}

void startADC(){
	HAL_ADC_Start(&hadc1);
}

