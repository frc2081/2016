#include "WPILib.h"
#include <iostream>
#include <string>

class Robot: public IterativeRobot {
private:
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";	// Default auto name
	const std::string autoNameCustom = "My Auto";	// Custom auto name
	std::string autoSelected; // What auto is selected

	Joystick *stick; 	// Pointer to a joystick
	Joystick *stick2;
	RobotDrive *drive;	// Pointer to a drive
	JoystickButton *buttonA; // All the button pointers
	JoystickButton *buttonB;
	JoystickButton *buttonX;
	JoystickButton *buttonY;
	JoystickButton *buttonLB;
	JoystickButton *buttonRB;
	JoystickButton *buttonBack;
	JoystickButton *buttonStart;
	JoystickButton *buttonLS;
	JoystickButton *buttonRS; 
	
	JoystickButton *buttonA2;
	JoystickButton *buttonB2;
	JoystickButton *buttonX2;
	JoystickButton *buttonY2;
	JoystickButton *buttonLB2;
	JoystickButton *buttonRB2;
	JoystickButton *buttonBack2;
	JoystickButton *buttonStart2;
	JoystickButton *buttonLS2;
	JoystickButton *buttonRS2; // End button pointers

	Encoder *LEnc;		// Pointer to left encoder
	Encoder *REnc;		// Pointer to right encoder
	Encoder *ArmEnc;	// Pointer to Arm encoder

	DigitalInput *PhoSen;

	VictorSP *winchmot; // PROPERLY NAMED pointer to winch motor
	
	Compressor *compress; // Pointer to compressor
	

	bool bA, bB, bX, bY, bLB, bRB, bBack, bStart, bLS, bRS, bA2, bB2, bX2, bY2, bLB2, bRB2, bBack2, bStart2, bLS2, bRS2; // Booleans on the states of each button
	
	// Trigger variable values
	float RTrig;
	float LTrig;
	float Trig;
	
	// Declaring variables for joystick axes
	double LaxisX, LaxisY;
	double RaxisX, RaxisY;
	
	bool yn;
	DoubleSolenoid *sArm = new DoubleSolenoid(0, 1);	// Solenoid for the opening and closing of the arms
	DoubleSolenoid *sLifter = new DoubleSolenoid(6, 7);	// Solenoid for lifting up the robot
	DoubleSolenoid *sPoker = new DoubleSolenoid(2, 3);	// Solenoid for the poker
	DoubleSolenoid *sLever = new DoubleSolenoid(4, 5);	// Solenoid to raise and lower the arms

enum states { // States for the auto load sequence
		IDLE,
		MV_TO_CAP,
		WT_FOR_BALL,
		HOLD_BALL,
		UNLOAD
	};
	int currentState  = IDLE;
	bool arms, lever, poker, lifter; // Varaibles to display where their respective parts are
	bool sensor;


	RobotDrive *drive; // Pointer to a drive
	JoystickButton *buttonA; // All the button pointers
	JoystickButton *buttonB;
	JoystickButton *buttonX;
	JoystickButton *buttonY;
	JoystickButton *buttonLB;
	JoystickButton *buttonRB;
	JoystickButton *buttonBack;
	JoystickButton *buttonStart;
	JoystickButton *buttonLS;
	JoystickButton *buttonRS; // End button pointers

	Encoder *LEnc;		// Pointer to left encoder
	Encoder *REnc;		// Pointer to right encoder
	Encoder *ArmEnc;	// Pointer to Arm encoder

	VictorSP *winchmot; // PROPERLY NAMED pointer to winch motor

	Compressor *compress; // Pointer to compressor

	// Solenoid pointers
	Solenoid *sArm1 = new Solenoid(0);
	Solenoid *sArm2 = new Solenoid(1);
	Solenoid *sPoker = new Solenoid(2);
	Solenoid *sLever = new Solenoid(3);

	bool bA, bB, bX, bY, bLB, bRB, bBack, bStart, bLS, bRS; // Booleans on the states of each button

	// Trigger variable values
	float RTrig;
	float LTrig;
	float Trig;

	// Declaring variables for joystick axes
	double LaxisX, LaxisY;
	double RaxisX, RaxisY;

	// Not sure, ask Doug
	// float count;
	// float current;

	/*
	enum states {
		IDLE,
		MV_TO_CAP,
		WT_FOR_BALL,
		HOLD_BALL,
	};
	int currentState;
	int success;
	int start;
	*/

	void RobotInit()
	{

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

		PhoSen = new DigitalInput(6);


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

		// Built-in something stuff
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);

		// SmartDashboard thing of sorts
		SmartDashboard::PutData("Auto Modes", chooser);

		LEnc = new Encoder(0, 5, false, Encoder::EncodingType::k4X);	// New encoder instance (Left drive)
		ArmEnc = new Encoder(2, 6, false, Encoder::EncodingType::k4X);	// New encoder instance (Winch)
		REnc = new Encoder(1, 7, false, Encoder::EncodingType::k4X);	// New encoder instance (Right Drive)
		
		// Declare winch motor
		winchmot = new VictorSP(0);
		
		// Declare compressor
		compress = new Compressor(0);

		// Declare winch motor
		winchmot = new VictorSP(0);

		// Declare compressor
		compress = new Compressor(0);

		// currentState = IDLE;
		// success = 0;
		// start = 0;

		// Just ask Doug
		compress->SetClosedLoopControl(true);
		compress->Start();
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()

	{
		// Weird built-in thing for auto modes
		autoSelected = *((std::string*)chooser->GetSelected());
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit()
	{

	}

	void checkbuttons() {
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

	void TeleopPeriodic()
	{
		// Run function to check button values
		checkbuttons();

		yn = PhoSen->Get();
		if(yn == TRUE) {
			printf("Hello World!");
			// SmartDashboard::PutBoolean("Limit switch: ", yn);
		};

		// Get left joystick values
		LaxisX = stick->GetX();
		LaxisY = stick->GetY();

		// Get right joystick values
		RaxisX = stick->GetRawAxis(4);
		RaxisY = stick->GetRawAxis(5);

		// Apply joystick values to motors 0 and 1
		drive->ArcadeDrive(LaxisY, RaxisX);
		
		// Prints button inputs to logs [Removed]

		// Get trigger values
		// Get left joystick values
		LaxisX = stick->GetX();
		LaxisY = stick->GetY();

		// Get right joystick values
		RaxisX = stick->GetRawAxis(4);
		RaxisY = stick->GetRawAxis(5);

		// Apply joystick values to motors 0 and 1
		drive->ArcadeDrive(LaxisY, RaxisX);

		// Prints button inputs to logs [Removed]

		// Get trigger values
		RTrig = stick->GetRawAxis(3);
		LTrig = stick->GetRawAxis(2);
		
		//Math for trigger/winch thing

		// Math for trigger/winch thing
		LTrig *= -1;
		Trig = LTrig + RTrig;

		//Print out trigger values
		//printf("Right Trig: %.2f \n", RTrig);
		//printf("Left Trig: %.2f \n", LTrig);

		SmartDashboard::PutNumber("Winch", Trig);
		//Tell winch motor to do things based on value of Trig
		winchmot->Set(Trig);
		
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
				yn = false;
				arms = false;
				lever = false;
				poker = false;
				lifter = false;
				//Displays where the parts are on the robot
				SmartDashboard::PutBoolean("Arms: \n", arms);
				SmartDashboard::PutBoolean("Lever: \n", lever);
				SmartDashboard::PutBoolean("Lifter: \n", lifter);
				SmartDashboard::PutBoolean("Poker: \n", poker);
				yn = bA2; //Gets the value of the A button
				if (yn == true) { //If the A button is pressed, change state to MV_TO_CAP
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
				yn = PhoSen->Get(); //Gets value of the photo sensor
				if (yn == true) { //If photo sensor is tripped, close the arms and change state to HOLD_BALL
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
				yn = bA2; //Checks if the A button is pressed
				if (yn == true) { //If the A button is pressed, open and arms move them inside the robot, and go back to IDLE
					currentState = UNLOAD;
				}
				if (bStart2 == true) { //If start button is pressed, move to idle state
					currentState = IDLE;
					yn = false;
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
				yn = PhoSen->Get(); //Checks if the photo sensor has been tripped
				if (yn != true) { //If the photo sensor is not tripped, set state to IDLE
					currentState = IDLE;
					yn = false;
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

		/*
		switch (currentState) {
		case IDLE:
			break;
		case MV_TO_CAP:
			break;
		case WT_FOR_BALL:
			break;
		case HOLD_BALL:
			break;
		}
		*/
	}

	void TestPeriodic()
	{

	}
};

//Start robot 

//Start robot
START_ROBOT_CLASS(Robot)
