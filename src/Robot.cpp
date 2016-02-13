#include "Robot.h"

void Robot::RobotInit()
{
	currentState = IDLE;

	// Declare new Joysticks
	stick = new Joystick(0);
	stick2 = new Joystick(1);

	// Declate buttons based on what button they literally are
	buttonA = new JoystickButton(stick, 1),
	buttonB = new JoystickButton(stick, 2),
	buttonX = new JoystickButton(stick, 3),
	buttonY = new JoystickButton(stick, 4),
	buttonLB = new JoystickButton(stick, 5),
	buttonRB = new JoystickButton(stick, 6),
	buttonBack = new JoystickButton(stick, 7),
	buttonStart = new JoystickButton(stick, 8),
	buttonLS = new JoystickButton(stick, 9),
	buttonRS = new JoystickButton(stick, 10);

	buttonA2 = new JoystickButton(stick2, 1),
	buttonB2 = new JoystickButton(stick2, 2),
	buttonX2 = new JoystickButton(stick2, 3),
	buttonY2 = new JoystickButton(stick2, 4),
	buttonLB2 = new JoystickButton(stick2, 5),
	buttonRB2 = new JoystickButton(stick2, 6),
	buttonBack2 = new JoystickButton(stick2, 7),
	buttonStart2 = new JoystickButton(stick2, 8),
	buttonLS2 = new JoystickButton(stick2, 9),
	buttonRS2 = new JoystickButton(stick2, 10);

	//Solenoids
	sArm = new DoubleSolenoid(0, 1);	// Solenoid for the opening and closing of the arms
	sLifter = new DoubleSolenoid(6, 7);	// Solenoid for lifting up the robot
	sPoker = new DoubleSolenoid(2, 3);	// Solenoid for the poker
	sLever = new DoubleSolenoid(4, 5);	// Solenoid to raise and lower the arms

	//Encoders
	LEnc = new Encoder(0, 1, false, Encoder::EncodingType::k4X);	// New encoder instance (Left drive)
	ArmEnc = new Encoder(2, 3, false, Encoder::EncodingType::k4X);	// New encoder instance (Winch)
	REnc = new Encoder(4, 5, false, Encoder::EncodingType::k4X);	// New encoder instance (Right Drive)
	ArmEnc->SetDistancePerPulse(ducksperpulse); //Sets distance per pulse IN INCHES
	LEnc->SetDistancePerPulse(ducksperpulse);
	REnc->SetDistancePerPulse(ducksperpulse);

	lmotor = new VictorSP(0);
	rmotor = new VictorSP(1);
	winchmot = new VictorSP(3);

	//Inputs
	RaFin = new AnalogInput (3);
	PhoSen = new DigitalInput(6);

	//Compressor
	compress = new Compressor(0);
	compress->SetClosedLoopControl(true);
	compress->Start();

	winchHold = 0.12;
	direction = true;

	averageGyro = 1.5;
	gyroCalibrate = 0;
	gyro = new ADXRS450_Gyro();
	gyro->Reset();
	gyro->Calibrate();

	atDefense = FALSE;
	crossedDefense = FALSE;

	// Declare new drive on PWM's 0 and 1
	drive = new RobotDrive(lmotor, rmotor);
}

void Robot::AutonomousInit()
{
	gyro->Reset();
}

void Robot::AutonomousPeriodic()
{
	Ltest += 2.5;
	Rtest += 2.4;
	SmartDashboard::PutNumber("Ltest:", Ltest);
	SmartDashboard::PutNumber("Rtest:", Rtest);
	//if((LEnc->Get() < 850) && (REnc->Get() < 850)) {
	if((Ltest < 850) && (Rtest < 850)) {
		drive->Drive(1, 0);
	} else {
		atDefense= TRUE;
	}
	//Autonomous code for defenses
	if(autoDefense == FRENCHTHING && atDefense == TRUE) {
		//Can't do
	}
	if(autoDefense == DRAWBRIDGE && atDefense == TRUE) {
		//Can't do
	}
	if(autoDefense == PORTCULLIS && atDefense == TRUE) {
		//Not sure of motors needed
	}
	if(autoDefense == SALLYPORT && atDefense == TRUE) {
		//Not sure of motors needed
	}
	if(autoDefense == MOAT && atDefense == TRUE) {
		if(LEnc->Get() < 310 && LEnc->Get() < 310) {
			drive->Drive(0.9, 0);
		}
	}
	if(autoDefense == RAMPART && atDefense == TRUE) {
		if(LEnc->Get() < 310 && LEnc->Get() < 310) {
			drive->Drive(0.9, 0);
		}

	}
	if(autoDefense == ROCKWALL && atDefense == TRUE) {
		if(LEnc->Get() < 310 && LEnc->Get() < 310) {
			drive->Drive(0.9, 0);
		}

	}
	if(autoDefense == ROUGHT && atDefense == TRUE) {
		if(LEnc->Get() < 310 && LEnc->Get() < 310) {
			drive->Drive(0.9, 0);
		}

	}
	if(autoDefense == LOWBAR && atDefense == TRUE) {
		if(LEnc->Get() < 310 && LEnc->Get() < 310) {
			drive->Drive(0.9, 0);
		}

	}
	//Autonomous code for positioning
	if(autoPosition == 1 && crossedDefense == TRUE) {

	}
	if(autoPosition == 2 && crossedDefense == TRUE) {

	}
	if(autoPosition == 3 && crossedDefense == TRUE) {

	}
	if(autoPosition == 4 && crossedDefense == TRUE) {

	}
	if(autoPosition == 5 && crossedDefense == TRUE) {

	}
	drive->ArcadeDrive(autoLeftMot, autoRightMot);
	lmotor->Set(lmotor->Get() * motorCorrectionValue);
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
	//Range Finder Math
	float Vm = RaFin->GetVoltage();
	float range = (Vm*1000)*((5/4.88)*.03937);
	SmartDashboard::PutNumber("Ultrasonic", range);

	//Update all joystick buttons
	checkbuttons();
	ArmEncValue = ArmEnc->Get();
	SmartDashboard::PutNumber("Gyro Calibration: ", gyroCalibrate);
	gyroAngle = gyro->GetAngle();

	// Get joystick values
	//Axes are swapped on xbox controllers....seems weird....
	//Hopefully this is correct?????
	RaxisY = stick->GetX();
	RaxisX = stick->GetY();
	LTrig = stick->GetRawAxis(2);
	RTrig = stick->GetRawAxis(3);
	LaxisY = stick->GetRawAxis(4);
	LaxisX = stick->GetRawAxis(5);

	if (bStart == true && bStartHold == false)
	{
		direction = !direction;
	}

	if(direction == false)
	{
		LaxisX *= -1;
		LaxisY *= -1;
		RaxisX *= -1;
		RaxisY *= -1;
	}
	//Get sensor inputs
	phoSensorVal = PhoSen->Get();

	//Math for winch thing
	//Combines both triggers into a single command for the winch motors
	LTrig *= -1;
	Trig = LTrig + RTrig;

	//SmartDashboard::PutNumber("Winch", Trig);

	//Automatic winch control
	if (bLB == true) //If left bumper on drive controller is held
	{
		setWinch = 0.5; //Set winch to extend at a certain power
		if (ArmEncValue >= 5000)
		{
			setWinch = 0; //When the winch hits the proper height, turn it off
		}
	}

	if (bRB == true) //If the right bumper on drive controller is held
	{
		setWinch = -0.5; //Set winch to retract at a certain power
		if (ArmEncValue <= 200)
		{
			setWinch = winchHold; //If the winch has raised the robot to a certian value, set the winch to the winch hold value and turn off the treads
			drive->Drive(0, 0);
		}
		if (ArmEncValue <= 500) //If the winch has raised the robot to a certian value, turn on the treads
		{
			drive->Drive(0.5, 0);
		}
	}

	/*
	arms = part of the robot that grabs the ball
	lever = part of robot that moves the arms inside and outside the robot
	poker = part of robot on front that extends outward
	lifter = part of robot that will extend beneath the robot to life it up

	ARMS
		true = open
		false = closed
	LEVER
		true = out of robot
		false = in robot
	POKER
		true = extended
		false = retracted
	LIFTER
		true = extended
		false = retracted

	DOUBLE SOLENOID CLASS
		kForward = true
		kReverse = false
	*/
	if (bLS2 == false)
	{
		switch (currentState)
		{
			case ENTER: //Entry state, nothing is commanded. Hit A to continue
				//Changed ENTER state to not advance to IDLE until a new button press of A is detected to ensure that
				//operator really intended to enter ball capture mode
				if (bA2 == true && bA2Hold == false) {currentState = IDLE;}
				break;

			case IDLE: //Idle state, nothing happens
				arms = true; //Arms open
				lever = true; //Arms out of robot
				poker = false; //Poker retracted
				lifter = false; //Lifter retracted

				//If the A button is pressed, change state to MV_TO_CAP
				if (bA2 == true) { currentState = MV_TO_CAP; }
				break;

			case MV_TO_CAP: //Moves arms into position and opens them
				arms = true; //Arms open
				lever = true; //Arms out of robot
				lifter = false; //Lifter retracted
				poker = false; //Poker retracted
				if (bA2 == true) { currentState = WT_FOR_BALL; } //Sets state to WT_FOR_BALL
				break;

			case WT_FOR_BALL: //Waiting for the ball to trip the photo sensor
				//Added new exit path to allow operator to stop trying to capture a ball by releasing the A button
				if (bA2 == false) { currentState = MV_TO_CAP; }
				//If photo sensor is tripped, close the arms and change state to HOLD_BALL
				if (phoSensorVal == true)
				{
					arms = false; //Arms closed
					lever = true; //Arms out
					poker = false; //Poker retracted
					lifter = false; //Lifter retracted
					currentState = HOLD_BALL;
				}
				//If start button is pressed, change to idle state
				if (bStart2 == true) { currentState = IDLE; }
				break;

			case HOLD_BALL: //Holds the ball in front of the robot
				arms = false; //Arms closed
				lever = true; //Arms out
				poker = false; //Poker retracted
				lifter = false; //Lifter retracted

				//If the A button is pressed, open and arms move them inside the robot, and go back to IDLE
				if (bA2 == true) { currentState = UNLOAD; }
				//If start button is pressed, move to idle state
				if (bStart2 == true) { currentState = IDLE; }
				break;

			case UNLOAD:
				arms = true; //Arms open
				lever = true; //Arms out
				poker = false; //Poker retracted
				lifter = false; //Lifter retracted

				//If the photo sensor is not tripped, set state to IDLE
				if (phoSensorVal != true) { currentState = IDLE; }
				break;
		}
	}

	//Manual mode controls engaged when left stick is held down
	if (bLS2 == true)
	{
			if (bY2 == true && bY2Hold == false)
			{
				lever = !lever;
			}
			//When A button is tapped, toggle the arms
			if (bA2 == true && bA2Hold == false)
			{
				arms = !arms;
			}
			//When B button is tapped, toggle the poker
			if (bB2 == true && bB2Hold == false)
			{
				poker = !poker;
			}
			//When X button is tapped, toggle the lifter
			if (bX2 == true && bX2Hold == false)
			{
				lifter = !lifter;
			}

			currentState = ENTER;
	}
	if (bY == true)
	{
		//When Y button is pressed, keep a minimum hold power applied to the winch. Otherwise, run winch like normal
		if (bRB == false) //If Y button is not pressed
		{
			setWinch = Trig; //Set winch power to the trigger value
		}
		else
		{
			if (Trig > winchHold) //If the trigger value is greater then 0.05, the winch hold value, set the winch power to the triggers
			{
				setWinch = Trig; //Set the value of the winch power to the value of the triggers
			}
			else //If the trigger value is less than the hold value, 0.05, set it to 0.05
			{setWinch = winchHold;}
		}
	}
	// Creates two integers: t and Tcurve
	//int t, Tcurve;
	// Multiplies trigger value by 100 to get percent
	//t = Trig * 100;
	// Creates parabolic throttle curve with equation of y=0.000001x^4
	//Tcurve = 0.000001 * pow(t, 4);
	// Creates linear throttle curve
	//Tcurve = abs(t);

	SmartDashboard::PutNumber("Winch", setWinch);
	//SmartDashboard::PutNumber("Edited", Tcurve);
	SmartDashboard::PutBoolean("Arms: \n", arms);
	SmartDashboard::PutBoolean("Lever: \n", lever);
	SmartDashboard::PutBoolean("Poker: \n", poker);
	SmartDashboard::PutBoolean("Lifter: \n", lifter);
	SmartDashboard::PutNumber("Gyro: \n", gyroAngle);
	SmartDashboard::PutNumber("Current State: ", currentState);
	SmartDashboard::PutNumber("Arm Encoder: ", ArmEncValue);
	if(tryingtofixmotor == 1) {
		double LEncval = LEnc->Get();
		double REncval = REnc->Get();
		SmartDashboard::PutNumber("Left motor encoder: \n", LEncval);
		SmartDashboard::PutNumber("Right motor encoder: \n", REncval);
	}

	if(lifter == true) {sLifter->Set(DoubleSolenoid::kForward);}
	else {sLifter->Set(DoubleSolenoid::kReverse);}
	if(arms == true) {sArm->Set(DoubleSolenoid::kForward);}
	else {sArm->Set(DoubleSolenoid::kReverse);}
	if(lever == true) {sLever->Set(DoubleSolenoid::kForward);}
	else {sLever->Set(DoubleSolenoid::kReverse);}
	if(poker == true) {sPoker->Set(DoubleSolenoid::kForward);}
	else {sPoker->Set(DoubleSolenoid::kReverse);}
	
	drive->ArcadeDrive(LaxisY, RaxisX);
	winchmot->Set(setWinch);

	if(tryingtofixmotor == 1) {
		double tempMotorVal = lmotor->Get();
		tempMotorVal *= motorCorrectionValue;
		lmotor->Set(tempMotorVal);
	}
}

void Robot::TestPeriodic()
{
}

void Robot::checkbuttons() {
	//Saves the previous value of the button
	bAHold = bA;
	bBHold = bB;
	bXHold = bX;
	bYHold = bY;
	bLBHold = bLB;
	bRBHold = bRB;
	bBackHold = bBack;
	bStartHold = bStart;
	bLSHold = bLS;
	bRSHold = bRS;

	bA2Hold = bA2;
	bB2Hold = bB2;
	bX2Hold = bX2;
	bY2Hold = bY2;
	bLB2Hold = bLB2;
	bRB2Hold = bRB2;
	bBack2Hold = bBack2;
	bStart2Hold = bStart2;
	bLS2Hold = bLS2;
	bRS2Hold = bRS2;

	// Check the states of all buttons (updates each loop of TeleopPeriodic)
	bA = stick->GetRawButton(1);
	bB = stick->GetRawButton(2);
	bX = stick->GetRawButton(3);
	bY = stick->GetRawButton(4);
	bLB = stick->GetRawButton(5);
	bRB = stick->GetRawButton(6);
	bBack = stick->GetRawButton(7);
	bStart = stick->GetRawButton(8);
	bLS = stick->GetRawButton(9);
	bRS = stick->GetRawButton(10);

	bA2 = stick2->GetRawButton(1);
	bB2 = stick2->GetRawButton(2);
	bX2 = stick2->GetRawButton(3);
	bY2 = stick2->GetRawButton(4);
	bLB2 = stick2->GetRawButton(5);
	bRB2 = stick2->GetRawButton(6);
	bBack2 = stick2->GetRawButton(7);
	bStart2 = stick2->GetRawButton(8);
	bLS2 = stick2->GetRawButton(9);
	bRS2 = stick2->GetRawButton(10);

}
//Start robot
START_ROBOT_CLASS(Robot)
