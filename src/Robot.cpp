#include "WPILib.h"
#include <iostream>
#include <string>

class Robot: public IterativeRobot {
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	Joystick *stick;
	RobotDrive *drive;
	JoystickButton *button1; //A
	JoystickButton *button2; //B
	JoystickButton *button3; //X
	JoystickButton *button4; //Y
	JoystickButton *button5; //Left Bumper
	JoystickButton *button6; //Right Bumper
	JoystickButton *button7; //Back
	JoystickButton *button8; //Start
	JoystickButton *button9; //Left Stick Depress
	JoystickButton *button10; //Right Stick Depress

	void RobotInit()
	{

		stick = new Joystick(0);
		drive = new RobotDrive(0, 1);

		button1 = new JoystickButton(stick, 1),
		button2 = new JoystickButton(stick, 2),
		button3 = new JoystickButton(stick, 3),
		button4 = new JoystickButton(stick, 4),
		button5 = new JoystickButton(stick, 5),
		button6 = new JoystickButton(stick, 6),
		button7 = new JoystickButton(stick, 7),
		button8 = new JoystickButton(stick, 8),
		button9 = new JoystickButton(stick, 9),
		button10 = new JoystickButton(stick, 10);

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

	void TeleopPeriodic()
	{
		bool button1, button2, button3, button4, button5, button6, button7, button8, button9, button10;

		double LaxisX, LaxisY;

		double RaxisX, RaxisY;

		LaxisX = stick->GetX();
		LaxisY = stick->GetY();

		RaxisX = stick->GetTwist();
		RaxisY = stick->GetRawAxis(5);

		drive->ArcadeDrive(LaxisY, RaxisX);
		if(button1 == 1) {
			printf("A\n");
		}
		if(button2 == TRUE) {
			printf("B\n");
		}
		if(button3 == TRUE) {
			printf("X\n");
		}
		if(button4 == TRUE) {
			printf("Y\n");
		}
		if(button5 == TRUE) {
			printf("LBumper\n");
		}
		if(button6 == TRUE) {
			printf("RBumper\n");
		}
		if(button7 == TRUE) {
			printf("Back\n");
		}
		if(button8 == TRUE) {
			printf("Start\n");
		}
		if(button9 == TRUE) {
			printf("LStick\n");
		}
		if(button10 == TRUE) {
			printf("RStick\n");
		}

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
