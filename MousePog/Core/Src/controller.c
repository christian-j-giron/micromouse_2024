/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include "irs.h"

int8_t donePID = 0;
int delay = 1000;


void move(int16_t n) {

	// 710 on level
	setPIDGoalD(n * 625);


	int leftIR = readLeftIR();
	int rightIR = readRightIR();

	//if going forward, constantly do IR corrections
	if (leftIR > 1500){
		setPIDGoalA(0.06 * (leftIR - 1500));
	}
	else if (rightIR > 1200) {
		setPIDGoalA(-0.075 * (rightIR - 1200));
	}
	else {
		setPIDGoalA(4);
	}


	donePID = PIDdone();
	while (donePID == 0) {
		donePID = PIDdone();
	}

	HAL_Delay(delay);
	resetPID();


}

void decDelay(void) {
	delay = delay / 2;
}


void turn(int8_t n) {

	setPIDGoalD(0);
	setPIDGoalA(n * 425);

	while (PIDdone() == 0) {
		donePID = PIDdone();
	}

	resetPID();

}

void correct(void) {

	setPIDGoalD(-600);

	HAL_Delay(1000);
	resetPID();

	setPIDGoalD(190);

	int leftIR = readLeftIR();
	int rightIR = readRightIR();

	if (leftIR > 1550){
		setPIDGoalA(0.0775 * (leftIR - 1550));
	}
	else if (rightIR > 1150) {
		setPIDGoalA(-0.08 * (rightIR - 1150));
	}
	else{
		setPIDGoalA(0);
	}

	donePID = PIDdone();
	while (donePID == 0) {
		donePID = PIDdone();
	}

	HAL_Delay(1000);
	resetPID();
}
