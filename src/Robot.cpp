#include "WPILib.h"
#include <iostream>
#include <string>

class Robot: public IterativeRobot 
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

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

	bool bA, bB, bX, bY, bLB, bRB, bBack, bStart, bLS, bRS;

	void RobotInit()
	{

		stick = new Joystick(0);
		drive = new RobotDrive(0, 1);

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

		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
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

	void checkbuttons() {
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
		checkbuttons();
		double LaxisX, LaxisY;
		double RaxisX, RaxisY;

		LaxisX = stick->GetX();
		LaxisY = stick->GetY();

		RaxisX = stick->GetTwist();
		RaxisY = stick->GetRawAxis(5);

		drive->ArcadeDrive(LaxisY, RaxisX);
		//Prints button inputs to logs
		/*if(bA == TRUE) {
			printf("A\n");
		}
		if(bB == TRUE) {
			printf("B\n");
		}
		if(bX == TRUE) {
			printf("X\n");
		}
		if(bY == TRUE) {
			printf("Y\n");
		}
		if(bLB == TRUE) {
			printf("LBumper\n");
		}
		if(bRB == TRUE) {
			printf("RBumper\n");
		}
		if(bBack == TRUE) {
			printf("Back\n");
		}
		if(bStart == TRUE) {
			printf("Start\n");
		}
		if(bLS == TRUE) {
			printf("LStick\n");
		}
		if(bRS == TRUE) {
			printf("RStick\n");
		}*/

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
