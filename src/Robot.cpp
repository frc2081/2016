#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	enum states { //States for the auto load sequence
		IDLE,
		MV_TO_CAP,
		WT_FOR_BALL,
		HOLD_BALL,
	};
	int currentState  = IDLE;
	Compressor *c; //Creates the compressor object
	Solenoid *sArm1 = new Solenoid(0); //Solenoid for the opening and closing of the arms
	Solenoid *sArm2 = new Solenoid(1); //Solenoid for the opening and closing of the arms
	Solenoid *sPoker = new Solenoid(2); //Solenoid for the poker
	Solenoid *sLever = new Solenoid(3); //Solenoid to raise and lower the arms
	float current; //Current state the state machine is on
	bool yn; //Varaible to hold true or false for the state machine
	Joystick *stick;
		RobotDrive *drive;
		JoystickButton *buttonA;
		JoystickButton *buttonB;
		JoystickButton *buttonX;
		JoystickButton *buttonY;
		JoystickButton *buttonLB;
		JoystickButton *buttonRB;
		JoystickButton *buttonBack;
		JoystickButton *buttonStart;
		JoystickButton *buttonLS;
		JoystickButton *buttonRS;
	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		currentState = IDLE; //Sets current state to IDLE, where nothing happens
		c = new Compressor(0); //Tells the robot where the compressor is plugged in
		//Starts the compressor
		c->SetClosedLoopControl(true);
		c->Start();
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
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		//Start  of the state machine that manages the auto load sequence
		switch (currentState) {
		case IDLE: //Idle state, nothing happens
			sLever->Set(false); //Keeps arms in the robot
			yn = buttonA->GetRawButton(); //Gets the value of the A button
			if (yn == true) { //If the A button is pressed, change state to MV_TO_CAP
				currentState = MV_TO_CAP;
			}
			break;
		case MV_TO_CAP: //Moves arms into posistion and opens them
			sLever->Set(true); //Lowers the arms
			sArm1->Set(false); //Opens the arms
			sArm2->Set(false); //Opens the arms
			currentState = WT_FOR_BALL; //Sets state to WT_FOR_BALL
			break;
		case WT_FOR_BALL: //Waiting for the ball to trip the photo sensor
			yn = PhoSen->Get(); //Gets value of the photo sensor
			if (yn == true) { //If photo sensor is tripped, close the arms and changee state to HOLD_BALL
				sArm1->Set(true);
				sArm2->Set(true);
				currentState = HOLD_BALL;
			}
			break;
		case HOLD_BALL: //Holds the ball in front of the robot
			yn = buttonA->GetRawButton(); //Checks if the A button is pressed
			if (yn == true) { //If the A button is pressed, open and arms move them inside the robot, and go back to IDLE
				sArm1->Set(false);
				sArm2->Set(false);
				sLever->Set(false);
				currentState = IDLE;
			}
			break;
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
