/*
 * encoders.c
 */

#include "main.h"
#include "encoders.h"


int16_t getRightEncoderCounts() {
	return (int16_t) TIM2->CNT;
}


int16_t getLeftEncoderCounts() {
	return (int16_t) TIM1->CNT * -1;
}


void resetEncoders() {
	TIM1->CNT = (int16_t) 0;
	TIM2->CNT = (int16_t) 0;
}
