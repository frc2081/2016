#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	Encoder*LEnc;
	Encoder*REnc;
	Encoder*ArmEnc;
	DigitalInput*PhoSen;
	float count;
	bool yn;

	void RobotInit()

	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		LEnc = new Encoder(0, 5, false, Encoder::EncodingType::k4X); // New encoder instance (Left drive)
		ArmEnc = new Encoder(2, 3, false, Encoder::EncodingType::k4X); // New encoder instance (Winch)
		REnc = new Encoder(4, 5, false, Encoder::EncodingType::k4X); // New encoder instance (Right Drive)
		PhoSen = new DigitalInput (6);
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
	{		yn = PhoSen->Get();
	while (yn == true) {
		printf("Hello World");
		Wait(10);
		lw->Run();}

	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

