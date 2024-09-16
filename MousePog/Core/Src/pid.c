/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "irs.h"

//PID constants: remain unchanged.
float kPw = 0.005;
float kDw = 0.001;

float kPx = 0.001;
float kDx = 0.0018;

//minimum for motor to run
float minv = 0.3;

//cap for distanceCorrection
float maxDistanceCorrection = 0.5;

//acceleration variables
float lv = 0; //left motor voltage
float plv = 0; // previous left motor voltage
float rv = 0; //right motor voltage
float prv = 0; //previous right motor voltage
float accelLimit = 1;

//Goal position set by driver code.
float goalDistance = 0;
float goalAngle = 0;

//Errors and corrections computed by updatePID()
float angleError = 0;
float oldAngleError = 0;
float angleCorrection;
float distanceError = 0;
float oldDistanceError = 0;
float distanceCorrection;

//Keeping track of when our error is negligible and for how long (in ticks).
int errorZero = 0;
const int EZT_INIT = 0;
int errorZeroTicks = EZT_INIT;
int ticksSinceUpdate = 0;
int8_t PIDisDone = 0;



// Extraneous test variable.
int counter = 0;

void resetPID() {

	angleError = 0;
	oldAngleError = 0;
	angleCorrection = 0;
	distanceError = 0;
	oldDistanceError = 0;
	distanceCorrection = 0;

	resetMotors();
	resetEncoders();

	goalDistance = 0;
	goalAngle = 0;

}

void updatePID() {

	counter++;
	angleError = goalAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
	oldAngleError = angleError;

	distanceError = goalDistance - (getLeftEncoderCounts() + getRightEncoderCounts()) / 2.0;
	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	oldDistanceError = distanceError;



	if (angleError < 2.5 && angleError > -2.5 && distanceError < 5.0 && distanceError > -5.0){
		errorZero = 1;
	}
	else {
		errorZero = 0;
	}

	if (angleError < 5 && angleError > -5){
		angleError = 0;
	}

	if (errorZero == 1) {
		errorZeroTicks++;
	}
	else {
		errorZeroTicks = EZT_INIT;
	}

	//cap angleCorrection
	if (angleCorrection > 0.5) {
		angleCorrection = 0.5;
	}
	if (angleCorrection < -0.5) {
			angleCorrection = -0.5;
	}

	//cap distanceCorrection
	if (distanceCorrection > 0.5) {
		distanceCorrection = 0.5;
	}
	if (distanceCorrection < -0.5) {
		distanceCorrection = -0.5;
	}

	lv = distanceCorrection + angleCorrection;
	rv = distanceCorrection - angleCorrection;

	// Acceleration limiting

//	if (lv > plv) { //left motor speeding up
//		if (lv - plv > accelLimit) {
//			lv = plv + accelLimit;
//		}
//	}
//	else { //left motor slowing down
//		if (plv - lv > accelLimit) {
//			lv = plv - accelLimit;
//		}
//	}
//
//	if (rv > prv) { //right motor speeding up
//		if (rv - prv > accelLimit) {
//			rv = prv + accelLimit;
//		}
//	}
//	else { //right motor slowing down
//		if (prv - rv > accelLimit) {
//			rv = prv - accelLimit;
//		}
//	}


	// Ensure minimum motor PWM
	if (lv > 0 && lv < minv) {
		lv = minv;
	}
	if (lv < 0 && lv > -minv) {
		lv = -minv;
	}
	if (rv > 0 && rv < minv) {
		rv = minv;
	}
	if (rv < 0 && rv > -minv) {
		rv = -minv;
	}


	setMotorLPWM(lv);
	setMotorRPWM(rv);

	plv = lv;
	prv = rv;

	ticksSinceUpdate++;


}

void setPIDGoalD(int16_t distance) {

	goalDistance = distance;
	errorZeroTicks = EZT_INIT;
	errorZero = 0;
	ticksSinceUpdate = 0;

}

void setPIDGoalA(int16_t angle) {

	goalAngle = angle;
	errorZeroTicks = EZT_INIT;
	errorZero = 0;
	ticksSinceUpdate = 0;
}

int8_t PIDdone(void) { // There is no bool type in C. True/False values are represented as 1 or 0.
	/*
	 * PIDdone() return true only if the error has been sufficiently close to zero for a certain number (100) of SysTick calls in a row.
	 */


	if (errorZeroTicks > 100 || ticksSinceUpdate > 1500) {
		return 1;
	}

	return 0;

}
