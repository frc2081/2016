/*
 * init.h
 *
 *  Created on: Jan 28, 2016
 *      Author: Matthew
 */

#ifndef INIT_H_
#define INIT_H_
#define ducksperpulse 0.08707563025
#define ducksinawinch 0.08707563025
#define tryingtofixmotor 1 //(Dis)enables the motor correction code- 1 means it will run the correction code
#define motorCorrectionValue 0.99 //Value the left motor will be mu
#include "WPILib.h"
#include <string>
#include <iostream>

class Robot: public IterativeRobot {
private:
	Joystick *stick; // Joystick plugged in first
	Joystick *stick2; //Joystick plugged in second

	RobotDrive *drive;	// Pointer to a drive

	JoystickButton *buttonA; // All the button pointers!
	JoystickButton *buttonB;
	JoystickButton *buttonX;
	JoystickButton *buttonY;
	JoystickButton *buttonLB;
	JoystickButton *buttonRB;
	JoystickButton *buttonBack;
	JoystickButton *buttonStart;
	JoystickButton *buttonLS;
	JoystickButton *buttonRS;
	JoystickButton *buttonA2; // Button pointers for joystick 2
	JoystickButton *buttonB2;
	JoystickButton *buttonX2;
	JoystickButton *buttonY2;
	JoystickButton *buttonLB2;
	JoystickButton *buttonRB2;
	JoystickButton *buttonBack2;
	JoystickButton *buttonStart2;
	JoystickButton *buttonLS2;
	JoystickButton *buttonRS2; // End button pointers

	Encoder *LEnc; // Pointer to left encoder
	Encoder *REnc; // Pointer to right encoder
	Encoder *ArmEnc; // Pointer to Arm encoder

	DoubleSolenoid *sArm;
	DoubleSolenoid *sLifter;
	DoubleSolenoid *sPoker;
	DoubleSolenoid *sLever;

	AnalogInput *RaFin;

	DigitalInput *PhoSen;

	VictorSP *winchmot; // PROPERLY NAMED pointer to winch motor
	VictorSP *lmotor; //New fancy motor thing for that really annoying drift thing because the motors are different slightly
	VictorSP  *rmotor;

	Compressor *compress; // Pointer to compressor

	ADXRS450_Gyro *gyro;
	float gyroAngle;
	float gyroAngle2;
	float gyroAngle3;
	float gyroAngle4;
	float averageGyro;
	float gyroCalibrate;

	bool bA, bB, bX, bY, bLB, bRB, bBack, bStart, bLS, bRS, bA2, bB2, bX2, bY2, bLB2, bRB2, bBack2, bStart2, bLS2, bRS2; // Booleans on the states of each button
	bool bAHold, bBHold, bXHold, bYHold, bLBHold, bRBHold, bStartHold, bBackHold, bLSHold, bRSHold;
	bool bA2Hold, bB2Hold, bX2Hold, bY2Hold, bLB2Hold, bRB2Hold, bBack2Hold, bStart2Hold, bLS2Hold, bRS2Hold;

	float RTrig, LTrig, Trig; // Trigger variable values

	double LaxisX, LaxisY; // Declaring variables for joystick axes
	double RaxisX, RaxisY;

	bool yn; // Boolean for state machine
	bool arms, lever, poker, lifter, phoSensorVal; // Variables to display where their respective parts are
	bool sensor;
	bool direction; //Holds current "front" direction of robot. False = ball grabber is front, true = track angle is front

	float setWinch, winchHold, ArmEncValue;

	//float lmotspeed, rmotspeed, lmotread, rmotread;

	enum states {
		ENTER,
		IDLE,
		MV_TO_CAP,
		WT_FOR_BALL,
		HOLD_BALL,
		UNLOAD
	};
	states currentState;
	void RobotInit(); // Scopes/initialization for robot functions
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void checkbuttons();
	void TeleopPeriodic();
	void TestPeriodic();
};

#endif /* INIT_H_ */
