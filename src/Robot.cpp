#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	enum states {
		IDLE,
		MV_TO_CAP,
		WT_FOR_BALL,
		HOLD_BALL,
	};
	int currentState = IDLE;
	Compressor *c;
	Solenoid *sArm1 = new Solenoid(0);
	Solenoid *sArm2 = new Solenoid(1);
	Solenoid *sPoker = new Solenoid(2);
	Solenoid *sLever = new Solenoid(3);
	float current;
	bool yn;
	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		currentState = IDLE;
		c = new Compressor(0);
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
		sArm1->Set(true);
		sArm2->Set(true);
		sPoker->Set(true);
		sLever->Set(true);
//		sArm1->Set(false);
//		sArm2->Set(false);
//		sPoker->Set(false);
//		sLever->Set(false);


		switch (currentState) {
		case IDLE:
			sLever->Set(false);
			yn = buttonA->GetRawButton();
			if (yn == true) {
				currentState = MV_TO_CAP;
			}
			break;
		case MV_TO_CAP:
			sLever->Set(true);
			sArm1->Set(false);
			sArm2->Set(false);
			currentState = WT_FOR_BALL;
			break;
		case WT_FOR_BALL:
			yn = PhoSen->Get();
			if (yn == true) {
				sArm1->Set(true);
				sArm2->Set(true);
				currentState = HOLD_BALL;
			}
			break;
		case HOLD_BALL:
			yn = buttonA->GetRawButton();
			if (yn == true) {
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
