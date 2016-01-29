#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	Encoder *winchEnc;
	//ADXRS450_Gyro *gyro;
	AnalogInput *rangeFinder;
	float range, voltToRangeConv;

	//This is Dan's Branch

	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		//SmartDashboard::PutData("Auto Modes", chooser);
		winchEnc = new Encoder(0,1);
		//gyro = new ADXRS450_Gyro();
		//gyro->Calibrate();
		rangeFinder = new AnalogInput(0);
		range = 0;
		voltToRangeConv = 1.0238;

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
		range = rangeFinder->GetVoltage()*voltToRangeConv;
		//SmartDashboard::PutNumber("Winch Encoder", winchEnc->GetDistance());
		//SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
		SmartDashboard::PutNumber("Range in volts", rangeFinder->GetVoltage());
		SmartDashboard::PutNumber("Range in mm", range*1000);
		SmartDashboard::PutNumber("Range in m", range);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
