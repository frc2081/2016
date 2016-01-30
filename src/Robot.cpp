#include "Robot.h"

void Robot::RobotInit()
{
	currentState = IDLE;
	// Declare new Joystick from (USB port?) 0
	stick = new Joystick(0);
	stick2 = new Joystick(1);

	// Declare new drive on PWM's 0 and 1
	drive = new RobotDrive(0, 1);

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
	LEnc = new Encoder(0, 5, false, Encoder::EncodingType::k4X);	// New encoder instance (Left drive)
	ArmEnc = new Encoder(2, 6, false, Encoder::EncodingType::k4X);	// New encoder instance (Winch)
	REnc = new Encoder(1, 7, false, Encoder::EncodingType::k4X);	// New encoder instance (Right Drive)
	ArmEnc->SetDistancePerPulse(ducksperpulse); //Sets distance per pulse IN INCHES
	LEnc->SetDistancePerPulse(ducksperpulse);
	REnc->SetDistancePerPulse(ducksperpulse);

	winchmot = new VictorSP(0);

	compress = new Compressor(0);
	compress->SetClosedLoopControl(true);
	compress->Start();

	PhoSen = new DigitalInput(6);
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
	// Run function to check button values
	checkbuttons();

	// Get joystick values
	LaxisX = stick->GetX();
	LaxisY = stick->GetY();
	RaxisX = stick->GetRawAxis(4);
	RaxisY = stick->GetRawAxis(5);
	RTrig = stick->GetRawAxis(3);
	LTrig = stick->GetRawAxis(2);

	//Get sensor inputs
	Armyn = PhoSen->Get();

	// Apply joystick values to motors 0 and 1
	drive->ArcadeDrive(LaxisY, RaxisX);

	// Math for trigger/winch thing
	LTrig *= -1;
	Trig = LTrig + RTrig;
	SmartDashboard::PutNumber("Winch", Trig);
//When Y button is pressed, keep a minimum hold power applied to the winch. Otherwise, run winch like normal
		if (bY != true) //If Y button is not pressed
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
	float currentStateP = currentState;
	//Start  of the state machine that manages the auto load sequence
	SmartDashboard::PutNumber("Current State: ", currentStateP);
	sensor = PhoSen->Get();
	SmartDashboard::PutBoolean("Switch: \n", sensor);
	if (bLS2 != true) { //If the Y button is pressed, go to manual control
		switch (currentState) {
		case IDLE: //Idle state, nothing happens
			sLever->Set(DoubleSolenoid::kReverse); //Keeps arms in the robot
			sArm->Set(DoubleSolenoid::kReverse); //Closes arms
			sPoker->Set(DoubleSolenoid::kReverse); //Keeps poker in robot
			sLifter->Set(DoubleSolenoid::kReverse); //Keeps lifter in the robot
			Armyn = false;
			arms = false;
			lever = false;
			poker = false;
			lifter = false;
			//Displays where the parts are on the robot
			SmartDashboard::PutBoolean("Arms: \n", arms);
			SmartDashboard::PutBoolean("Lever: \n", lever);
			SmartDashboard::PutBoolean("Lifter: \n", lifter);
			SmartDashboard::PutBoolean("Poker: \n", poker);
			Armyn = bA2; //Gets the value of the A button
			if (Armyn == true) { //If the A button is pressed, change state to MV_TO_CAP
				currentState = MV_TO_CAP;
			}
			break;
		case MV_TO_CAP: //Moves arms into posistion and opens them
			sLever->Set(DoubleSolenoid::kForward); //Lowers the arms
			sArm->Set(DoubleSolenoid::kReverse); //Opens the arms
			arms = false;
			lever = true;
			lifter = false;
			poker = false;
			//Displays where the parts are on the robot
			SmartDashboard::PutBoolean("Arms: \n", arms);
			SmartDashboard::PutBoolean("Lever: \n", lever);
			SmartDashboard::PutBoolean("Poker: \n", poker);
			SmartDashboard::PutBoolean("Lifter: \n", lifter);
			currentState = WT_FOR_BALL; //Sets state to WT_FOR_BALL
			break;
		case WT_FOR_BALL: //Waiting for the ball to trip the photo sensor
				Armyn = PhoSen->Get(); //Gets value of the photo sensor
				if (Armyn == true) { //If photo sensor is tripped, close the arms and change state to HOLD_BALL
					sArm->Set(DoubleSolenoid::kForward); //Keeps arms open
					sLever->Set(DoubleSolenoid::kForward); //Keeps arms down
					arms = true;
					lever = true;
					poker = false;
					lifter = false;
					//Displays where the parts are on the robot
					SmartDashboard::PutBoolean("Arms: \n", arms);
					SmartDashboard::PutBoolean("Lever: \n", lever);
					SmartDashboard::PutBoolean("Poker: \n", poker);
					SmartDashboard::PutBoolean("Lifter: \n", lifter);
					currentState = HOLD_BALL;
				}
			if (bStart2 == true) { //If start button is pressed, change to idle state
				currentState = IDLE;
				yn = false;
			}
			break;
		case HOLD_BALL: //Holds the ball in front of the robot
				Armyn = bA2; //Checks if the A button is pressed
				if (Armyn == true) { //If the A button is pressed, open and arms move them inside the robot, and go back to IDLE
					currentState = UNLOAD;
				}
				if (bStart2 == true) { //If start button is pressed, move to idle state
					currentState = IDLE;
					Armyn = false;
				}
				sArm->Set(DoubleSolenoid::kReverse); //Keeps arms closed
				sLever->Set(DoubleSolenoid::kForward); //Keeps arms down
				arms = true;
				lever = true;
				poker = false;
				lifter = false;
				//Displays where the parts are on the robot
				SmartDashboard::PutBoolean("Arms: \n", arms);
				SmartDashboard::PutBoolean("Lever: \n", lever);
				SmartDashboard::PutBoolean("Poker: \n", poker);
				SmartDashboard::PutBoolean("Lifter: \n", lifter);
				break;
		case UNLOAD:
				sArm->Set(DoubleSolenoid::kForward); //Opens arms
				//sPoker->Set(DoubleSolenoid::kForward); //Extends poker
				sLever->Set(DoubleSolenoid::kForward); //Keeps arms down
				arms = true;
				lever = true;
				poker = false;
				lifter = false;
				//Displays where the parts are on the robot
				SmartDashboard::PutBoolean("Arms: \n", arms);
				SmartDashboard::PutBoolean("Lever: \n", lever);
				SmartDashboard::PutBoolean("Poker: \n", poker);
				SmartDashboard::PutBoolean("Lifter: \n", lifter);
				Armyn = PhoSen->Get(); //Checks if the photo sensor has been tripped
				if (Armyn != true) { //If the photo sensor is not tripped, set state to IDLE
					currentState = IDLE;
					Armyn = false;
				}
				break;
		}
	} else { //Manual control
		if (bRB2 == true) { //When left bumper is pressed, raise the arms
			sLever->Set(DoubleSolenoid::kReverse);
			lever = false;
			SmartDashboard::PutBoolean("Lever: \n", lever);
			}
		if (bLB2 == true) { //When right bumper is pressed, lower the arms
			sLever->Set(DoubleSolenoid::kForward);
			lever = true;
			SmartDashboard::PutBoolean("Lever: \n", lever);
		}
		if (bA2 != false) { //When A button held, open arms. Otherwise, close them
			sArm->Set(DoubleSolenoid::kForward);
			arms = true;
			SmartDashboard::PutBoolean("Arms: \n", arms);
		} else {
			sArm->Set(DoubleSolenoid::kReverse);
			arms = false;
			SmartDashboard::PutBoolean("Arms: \n", arms);
		}
		if (bB2 != false) { //When B button held, extend poker. Otherwise, retract it
			sPoker->Set(DoubleSolenoid::kForward);
			poker = true;
			SmartDashboard::PutBoolean("Poker: \n", poker);
		} else {
			sPoker->Set(DoubleSolenoid::kReverse);
			poker = false;
			SmartDashboard::PutBoolean("Poker: \n", poker);
		}
		if (bX2 != false) { //When X button held, extend lifter. Otherwise, retract it
			sLifter->Set(DoubleSolenoid::kForward);
			lifter = true;
			SmartDashboard::PutBoolean("Lifter: \n", lifter);
		} else {
			sLifter->Set(DoubleSolenoid::kReverse);
			lifter = false;
			SmartDashboard::PutBoolean("Lifter: \n", lifter);
			currentState = IDLE; //Sets the current state to IDLE after manual mode has been left
		}
		currentState = IDLE;
	}

	// Creates two integers: t and Tcurve
	int t, Tcurve;

	// Multiplies trigger value by 100 to get percent
	t = Trig * 100;

	// Creates parabolic throttle curve with equation of y=0.000001x^4
	//Tcurve = 0.000001 * pow(t, 4);
	// Creates linear throttle curve
	Tcurve = abs(t);

	SmartDashboard::PutNumber("Edited", Tcurve);
	// Tell winch motor to do things based on value of Trig
	winchmot->Set(Tcurve/100);
}

void Robot::TestPeriodic()
{

}

void Robot::checkbuttons() {
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
