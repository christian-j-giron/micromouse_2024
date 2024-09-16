/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
#include "encoders.h"

//PID constants: remain unchanged.
float kPw = 0.03;
float kDw = 0.0018;
float kPx = 0.001;
float kDx = 0.0018;

//minimum for motor to run
float minv = 0.4;

//acceleration stuff
float lv = 0; //left motor voltage
float plv = 0; // previous left motor voltage
float rv = 0; //right motor voltage
float prv = 0; //previous right motor voltage
float accelLimit = 1;

//Goal position set by driver code.
float goalDistance = 0;
int goalAngle = 0;

//Errors and corrections computed by updatePID()
int angleError = 0;
int oldAngleError = 0;
float angleCorrection;
float distanceError = 0;
float oldDistanceError = 0;
float distanceCorrection;

//Keeping track of when our error is negligible and for how long (in ticks).
int errorZero = 0;
int errorZeroTicks = 0;
int8_t PIDisDone = 0;

int counter = 0;

void resetPID() {
	/*
	 * For assignment 3.1: This function does not need to do anything
	 * For assignment 3.2: This function should reset all the variables you define in this file to help with PID to their default
	 *  values. You should also reset your motors and encoder counts (if you tell your rat to turn 90 degrees, there will be a big
	 * difference in encoder counts after it turns. If you follow that by telling your rat to drive straight without first
	 * resetting the encoder counts, your rat is going to see a huge angle error and be very unhappy).
	 *
	 * You should additionally set your distance and error goal values (and your oldDistanceError and oldAngleError) to zero.
	 */

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

	counter++;



}

void updatePID() {
	/*
	 * This function will do the heavy lifting PID logic. It should do the following: read the encoder counts to determine an error,
	 * use that error along with some PD constants you determine in order to determine how to set the motor speeds, and then actually
	 * set the motor speeds.
	 *
	 * For assignment 3.1: implement this function to get your rat to drive forwards indefinitely in a straight line. Refer to pseudocode
	 * example document on the google drive for some pointers
	 *
	 * TIPS (assignment 3.1): Create kPw and kDw variables, and use a variable to store the previous error for use in computing your
	 * derivative term. You may get better performance by having your kDw term average the previous handful of error values instead of the
	 * immediately previous one, or using a stored error from 10-15 cycles ago (stored in an array?). This is because systick calls so frequently
	 * that the error change may be very small and hard to operate on.
	 *
	 * For assignment 3.2: implement this function so it calculates distanceError as the difference between your goal distance and the average of
	 * your left and right encoder counts. Calculate angleError as the difference between your goal angle and the difference between your left and
	 * right encoder counts. Refer to pseudocode example document on the google drive for some pointers.
	 */


	angleError = goalAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
	oldAngleError = angleError;

	distanceError = goalDistance - (getLeftEncoderCounts() + getRightEncoderCounts()) / 2.0;
	distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	oldDistanceError = distanceError;

	if (abs(angleError) < 5 && abs(distanceError) < 5){
		errorZero = 1;
	}
	else {
		errorZero = 0;
	}

	if (angleCorrection < 1 && angleCorrection > -1){
		angleCorrection = 0;
	}

	if (errorZero == 1) {
		errorZeroTicks++;
	}
	else {
		errorZeroTicks = 0;
	}


	lv = distanceCorrection + angleCorrection;
	rv = distanceCorrection - angleCorrection;

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


	// Implement opposite motor correction.
	if (errorZero == 0) {
		if (lv > 0 && lv < minv) {
			//rv += (minv - lv);
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
	}

	setMotorLPWM(lv);
	setMotorRPWM(rv);

	plv = lv;
	prv = rv;


}

void setPIDGoalD(int16_t distance) {
	/*
	 * For assignment 3.1: this function does not need to do anything.
	 * For assignment 3.2: this function should set a variable that stores the goal distance.
	 */
	goalDistance = distance;
	errorZeroTicks = 0;
	errorZero = 0;

}

void setPIDGoalA(int16_t angle) {
	/*
	 * For assignment 3.1: this function does not need to do anything
	 * For assignment 3.2: This function should set a variable that stores the goal angle.
	 */
	goalAngle = angle;
	errorZeroTicks = 0;
	errorZero = 0;
}

int8_t PIDdone(void) { // There is no bool type in C. True/False values are represented as 1 or 0.
	/*
	 * For assignment 3.1: this function does not need to do anything (your rat should just drive straight indefinitely)
	 * For assignment 3.2:This function should return true if the rat has achieved the set goal. One way to do this by having updatePID() set some variable when
	 * the error is zero (realistically, have it set the variable when the error is close to zero, not just exactly zero). You will have better results if you make
	 * PIDdone() return true only if the error has been sufficiently close to zero for a certain number, say, 50, of SysTick calls in a row.
	 */

	counter++;


	if (errorZeroTicks > 100) {
		return 1;
	}

	return 0;

}
