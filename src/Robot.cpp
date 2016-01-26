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
	Joystick *stick;
	float count;
	VictorSP *mot;

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

		LEnc = new Encoder(0, 5, false, Encoder::EncodingType::k4X); // New encoder instance (Left drive)
		ArmEnc = new Encoder(2, 6, false, Encoder::EncodingType::k4X); // New encoder instance (Winch)
		REnc = new Encoder(1, 7, false, Encoder::EncodingType::k4X); // New encoder instance (Right Drive)
		stick = new Joystick(0);
		mot = new VictorSP(0);


		//currentState = IDLE;
		//success = 0;
		//start = 0;
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
		float RTrig;
		float LTrig;
		float Trig;

		RTrig = stick->GetRawAxis(3);
		LTrig = stick->GetRawAxis(2);
		LTrig *= -1;
		Trig = LTrig + RTrig;

		printf("Right Trig: %.2f \n", RTrig);
		printf("Left Trig: %.2f \n", LTrig);

		mot->Set(Trig);

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
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)

