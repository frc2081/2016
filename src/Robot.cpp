#include "WPILib.h"
#include <iostream>
#include <string>

class Robot: public IterativeRobot {
private:
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default"; //Default auto name
	const std::string autoNameCustom = "My Auto"; //Custom auto name
	std::string autoSelected; //What auto is selected

	Joystick *stick; //Pointer to a joystick
	RobotDrive *drive; //Pointer to a drive
	JoystickButton *buttonA; //All the button pointers
	JoystickButton *buttonB;
	JoystickButton *buttonX;
	JoystickButton *buttonY;
	JoystickButton *buttonLB;
	JoystickButton *buttonRB;
	JoystickButton *buttonBack;
	JoystickButton *buttonStart;
	JoystickButton *buttonLS;
	JoystickButton *buttonRS; //End button pointers

	Encoder *LEnc; //Pointer to left encoder
	Encoder *REnc; //Pointer to right encoder
	Encoder *ArmEnc; //Pointer to Arm encoder
	AnalogInput *PreSen;
	DigitalInput *PhoSen;

	VictorSP *winchmot; //PROPERLY NAMED pointer to winch motor
	
	Compressor *compress; //Pointer to compressor
	
	//Solenoid pointers
	Solenoid *sArm1 = new Solenoid(0);
	Solenoid *sArm2 = new Solenoid(1);
	Solenoid *sPoker = new Solenoid(2);
	Solenoid *sLever = new Solenoid(3);

	bool bA, bB, bX, bY, bLB, bRB, bBack, bStart, bLS, bRS; //Booleans on the states of each button
	
	bool pressGood;

	//Trigger variable values
	float RTrig;
	float LTrig;
	float Trig;
	
	//Declaring variables for joystick axes
	double LaxisX, LaxisY;
	double RaxisX, RaxisY;
	
	bool yn;

	//Not sure, ask Doug
	//float count;
	//float current;
	
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

		//Declare new Joystick from (USB port?) 0
		stick = new Joystick(0);
		
		//Declare new drive on PWM's 0 and 1
		drive = new RobotDrive(0, 1);

		//Declate buttons based on what button they literally are
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


		
		//SmartDashboard thing of sorts
		SmartDashboard::PutData("Auto Modes", chooser);


		LEnc = new Encoder(0, 5, false, Encoder::EncodingType::k4X); // New encoder instance (Left drive)
		ArmEnc = new Encoder(2, 6, false, Encoder::EncodingType::k4X); // New encoder instance (Winch)
		REnc = new Encoder(1, 7, false, Encoder::EncodingType::k4X); // New encoder instance (Right Drive)
		
		PreSen = new AnalogInput(0);
		PhoSen = new DigitalInput(2);

		//Declare winch motor
		winchmot = new VictorSP(0);
		
		//Declare compressor
		compress = new Compressor(0);

		//currentState = IDLE;
		//success = 0;
		//start = 0;

		//Just ask Doug
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
		//Weird built-in thing for auto modes
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom) {
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom) {
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void TeleopInit()
	{

	}

	void checkbuttons() {
		//Check the states of all buttons (updates each loop of TeleopPeriodic)
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
	}

	void TeleopPeriodic()
	{

		int Pres = PreSen->GetVoltage();
		//int Pres = 50*(Vout)-25;			// Vout is voltage and Pres is in terms of PSI (commented until tested)

		if(Pres>=45) {
			pressGood = true;
			SmartDashboard::PutBoolean("Pressure is Good!", pressGood);
		}
		// 45 psi is good
		SmartDashboard::PutNumber("Pressure: ", Pres);

		//Run function to check button values
		checkbuttons();

		yn = PhoSen->Get();
		if(yn == TRUE) {
			//SmartDashboar`d::PutBoolean("Limit switch: ", yn);
		};

		//Get left joystick values
		LaxisX = stick->GetX();
		LaxisY = stick->GetY();

		//Get right joystick values
		RaxisX = stick->GetRawAxis(4);
		RaxisY = stick->GetRawAxis(5);

		//Apply joystick values to motors 0 and 1
		drive->ArcadeDrive(LaxisY, RaxisX);
		
		//Prints button inputs to logs [Removed]

		//Get trigger values
		RTrig = stick->GetRawAxis(3);
		LTrig = stick->GetRawAxis(2);
		
		//Math for trigger/winch thing
		LTrig *= -1;
		Trig = LTrig + RTrig;

		//Print out trigger values
		//printf("Right Trig: %.2f \n", RTrig);
		//printf("Left Trig: %.2f \n", LTrig);

		//Tell winch motor to do things based on value of Trig
		winchmot->Set(Trig);
		
		/*
		 *
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
START_ROBOT_CLASS(Robot)
