#include "Robot.h"

void Robot::RobotInit()
{
	currentState = ENTER;

	// Declare new Joysticks
	stick = new Joystick(0);
	stick2 = new Joystick(1);

	cameras = new CAMERAFEEDS(stick);
	cameras->init();

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
	//1 is back PCM, 0 is front PCM(front at track angle)
	sArm = new DoubleSolenoid(0, 0, 1);	// Solenoid for the opening and closing of the arms
	sLifter = new DoubleSolenoid(0, 6, 7);	// Solenoid for lifting up the robot
	sPoker = new DoubleSolenoid(0, 2, 3);	// Solenoid for the poker
	sLever = new DoubleSolenoid(1, 0, 1);	// Solenoid to raise and lower the arms
	sWinch = new DoubleSolenoid(0, 4, 5); // Solenoid to raise the winch

	//Encoders
	LEnc = new Encoder(0, 1, false, Encoder::EncodingType::k4X);	// New encoder instance (Left drive)
	ArmEnc = new Encoder(2, 3, false, Encoder::EncodingType::k4X);	// New encoder instance (Winch)
	REnc = new Encoder(4, 5, false, Encoder::EncodingType::k4X);	// New encoder instance (Right Drive)
	ArmEnc->SetDistancePerPulse(ducksperpulse); //Sets distance per pulse IN INCHES
	LEnc->SetDistancePerPulse(ducksperpulse);
	REnc->SetDistancePerPulse(ducksperpulse);

	winchmot = new VictorSP(2);
	lmotor = new VictorSP(0);
	rmotor = new VictorSP(1);
	PreSen = new AnalogInput(0);
	PreSen->SetAverageBits(3);
	RaFin =  new AnalogInput (3);
	compress = new Compressor(0);
	compress->SetClosedLoopControl(true);
	compress->Start();

	PhoSen = new DigitalInput(6);
	winchHold = -0.12;
	direction = true;
	winchSol = false;

	averageGyro = 1.5;
	gyroCalibrate = 0;
	gyro = new ADXRS450_Gyro();
	gyro->Calibrate();
	gyro->Reset();
	
	/*while (gyroCalibrate < 5){
		while (averageGyro >= 1) {
			gyro->Calibrate();
			gyroAngle = gyro->GetAngle();
			Wait(100);
			gyroAngle2 = gyro->GetAngle();
			Wait(100);
			gyroAngle3 = gyro->GetAngle();
			Wait(100);
			gyroAngle4 = gyro->GetAngle();
			averageGyro = (gyroAngle + gyroAngle2 + gyroAngle3 + gyroAngle4) / 4;
			gyroCalibrate += 1;
		}
		if (averageGyro < 0.5) {
			break;
		}
	}*/
	
	// Declare new drive with our drive motors
	drive = new RobotDrive(lmotor, rmotor);
	lifter = true; //Lifter retracted
	arms = false; //Arms closed
	lever = false; //Arms inside of robot
	poker = false; //Poker retracted

	/*
	AutoMode Settings
	0 = no auto mode
	1 = move to defense but do not attempt to cross it
	2 = move to defense and cross it
	3 = move to defense, cross it, then attempt a goal. Works only in positions 1, 2, and 5
	*/
	autoMode = 0;		//autoMode is always initialized to do nothing
	autoPosition = 1; 	//Position is 1 indexed to match the field diagram
	autoDefense = LOWBAR;
	
	autoHighDrive = .8;
	autoLowDrive = .5;
	autoNavigationDrive = .5;
	autoArmMoveTime = 5; //duration of time arms need to move. units are # of software loops
	
	armClearDelay = 0; //when taking a shot, amount of time to wait for ball to settle after opening arms 
	drive->SetExpiration(.20);

	SmartDashboard::PutNumber("AutoDefense", 0);
	SmartDashboard::PutNumber("AutoPosition", 1);
	SmartDashboard::PutNumber("AutoMode", 0);
}

void Robot::AutonomousInit()
{
	//Saftey Feature - autoMode is set to 0 on start of autoInit so that auto will only run if the robot successfully pulls an auto command from the dashboard
	autoMode = 0;
	
	//Reset auto Variables in case this isn't the first time auto is run in the power cycle
	autoDistance = 0;
	autoDefenseDrivePower = 0;
	autoDrivePower = 0;
	autoTurnPower = 0;
	
	autoCurrentStep = MOVE_TO_DEFENSE;
	autoDelay = 0; //generic counter used to various delay tasks in auto mode
	
	//Reset sensors
	gyro->Reset();
	LEnc->Reset();
	REnc->Reset();	
	
	//Get autonoumous mode parameters from the dashboard 
	autoMode = SmartDashboard::GetNumber("AutoMode", 0);
	autoDefense = static_cast<Defense>(SmartDashboard::GetNumber("AutoDefense", 0));
	autoPosition = SmartDashboard::GetNumber("AutoPosition", 1);
	
	//The automated goal code only works in positions 1, 2 and 5, so
	//Automode 3 is not allowed in Positions 3 and 4;
	if(autoMode >= 3 && (autoPosition == 3 || autoPosition == 4))
	{
		autoMode = 2;
	}
	
	//Update drive here to prevent tripping the drive safety timout
	drive->Drive(0, 0);
	
	//Select drive power based on defense
	//RoughTerrain needs more power and speed to cross
	//Cheval must be approached in reverse
	//all other defenses are low power
	if(autoDefense == ROUGHTERRAIN)
	{
		autoDefenseDrivePower = autoHighDrive;
	}
	else if (autoDefense == CHEVAL)  {autoDefenseDrivePower = -autoLowDrive;}
	else{autoDefenseDrivePower = autoLowDrive;}	
	
	//Configure target angle for castle turn
	//Cheval and portcullis routines drive backward and so must have a different angle
	if(autoDefense == CHEVAL || autoDefense == PORTCULLIS) {autoCastleTargetAngle = 45;}
	else {autoCastleTargetAngle = -135;}
	
	//Announce to console that auto mode is beginning and the settings it is using
	printf("\n\n\n***********Initiating autonomous run*************\n");
	printf("Mode: %i Defense: %i Position: %i\n", autoMode, autoDefense, autoPosition);
	printf("**************************************************\n\n\n");
}

void Robot::AutonomousPeriodic()
{
	LEncVal = LEnc->GetDistance();
	REncVal = REnc->GetDistance();
	autoDistance = (LEncVal + REncVal) / 2;
	gyroAngle = gyro->GetAngle();
	autoDrivePower = 0;
	autoTurnPower = 0;
	
	printf("Step: %i Distance: %f rEnc: %f lEnc: %f gAngle %f\n", autoCurrentStep, autoDistance, REncVal, LEncVal, gyroAngle);

//***************AUTO MODE**********************************
	//Move to defense Step
	if(autoMode >= 1 && autoCurrentStep == MOVE_TO_DEFENSE)
	{				
		if(autoDistance < 60) 
		{
			autoDrivePower = autoDefenseDrivePower;
		}
		else 
		{	
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = DEFENSE_SPECIFIC;
			
			//Encoders are reset here so the drive in the next step has a good starting point
			LEnc->Reset();
			REnc->Reset();
			//When encoders are reset, values for the current loop must also be recalculated
			LEncVal = LEnc->GetDistance();
			REncVal = REnc->GetDistance();
			autoDistance = (LEncVal + REncVal) / 2;
		}
		
		//For some defenses, the arms need to be down. 
		//Put them down while moving to save time
		if(autoDefense == LOWBAR || autoDefense == PORTCULLIS)
		{
			lever = true;
		}
	}
	
//*****************************************************
	//Perform any actions specific to crossing a certain defense
	if(autoMode >= 2 && autoCurrentStep == DEFENSE_SPECIFIC)
	{		
		//CHEVAL actions - once robot has reached the defense, put arms down to push platform down
		if(autoDefense == CHEVAL)
		{
			lever = true;
			autoDelay++;
			autoDrivePower = 0;
			autoTurnPower = 0;
			
			if(autoDelay > autoArmMoveTime)
			{
				autoCurrentStep = CROSS_DEFENSE;
				autoDelay = 0;
			}
		} else
		//PORTCULLIS actions - raise arms up while slowly driving forward to raise the gate
		if(autoDefense == PORTCULLIS)
		{
			lever = false;
			autoDelay++;
			autoDrivePower = .3;
			autoTurnPower = 0;
			
			if(autoDelay > 10)
			{
				autoCurrentStep = CROSS_DEFENSE;
				autoDelay = 0;
			}
		}
		else {autoCurrentStep = CROSS_DEFENSE;}
	}
	
//*****************************************************
	//Cross defense step
	if(autoMode >= 2 && autoCurrentStep == CROSS_DEFENSE)
	{
		if(autoDistance < 120) 
		{
			autoDrivePower = autoDefenseDrivePower;	
		}
		else 
		{	
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = ALIGN_TO_ZERO;
		}
	}		

//*****************************************************	
	//Rotate back to 0 angle step
	//put arms back in robot for safety and to increase clearance
	if(autoMode >= 3 && autoCurrentStep == ALIGN_TO_ZERO)
	{
		lever = false;
		if(gyroAngle < -2) { autoTurnPower = autoTurnRate; }
		else if(gyroAngle > 2) { autoTurnPower = -autoTurnRate; }
		else 
		{	
			//Encoders are reset here so the drive in the next step has a good starting point
			LEnc->Reset();
			REnc->Reset();
			//When encoders are reset, values for the current loop must also be recalculated
			LEncVal = LEnc->GetDistance();
			REncVal = REnc->GetDistance();
			autoDistance = (LEncVal + REncVal) / 2;
	
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = MOVE_TO_CASTLE_TURN;
		}
	}		
	
//*****************************************************	
	//Drive to the point where the robot turns to face the goal
	if(autoMode >= 3 && autoCurrentStep == MOVE_TO_CASTLE_TURN)
	{
		if(autoDistance < 50) 
		{
			autoDrivePower = autoNavigationDrive;	
		}
		else 
		{	
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = CASTLE_TURN;
		}
	}		

//*****************************************************	
	//Turn to aim the ball at the goal
	//If Defense was CHEVAL or PORTCULLIS, robot is approching backwards and must turn differently
	//Otherwise, all turns are the same
	if(autoMode >= 3 && autoCurrentStep == CASTLE_TURN)
	{		
		if(gyroAngle > autoCastleTargetAngle + 2) { autoTurnPower = -autoTurnRate; }
		else if(gyroAngle < autoCastleTargetAngle -2) { autoTurnPower = autoTurnRate; }
		else 
		{	
			//Encoders are reset here so the drive in the next step has a good starting point
			LEnc->Reset();
			REnc->Reset();
			LEncVal = LEnc->GetDistance();
			REncVal = REnc->GetDistance();
			autoDistance = (LEncVal + REncVal) / 2;
			
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = MV_TO_CASTLE;
		}
	}	
	
//*****************************************************	
	//Move to the castle and put the arms down for a shot
	if(autoMode >= 3 && autoCurrentStep == MV_TO_CASTLE)
	{
		lever = true;
		if(autoDistance < 20) 
		{
			autoDrivePower = autoNavigationDrive;	
		}
		else 
		{	
			autoDrivePower = 0;
			autoTurnPower = 0;
			autoCurrentStep = AUTO_SHOOT;
		}
	}	

//*****************************************************	
	//Open the arms and then shoot until the end of auto
	if(autoMode >= 3 && autoCurrentStep == AUTO_SHOOT)
	{
		arms = true;
		poker = false;
		armClearDelay++;
		
		//Shoot every 15 loops
		if(armClearDelay >= 15)
		{	
			poker = true;
			armClearDelay = 0;
		}
	}		
//*****************************************************	
	//Only allow drive commands to be sent to the ouputs if an auto mode has been selected
	if(autoMode != 0)
	{
		drive->Drive(autoDrivePower, autoTurnPower);

		if(lifter == true) {sLifter->Set(DoubleSolenoid::kForward);}
		else {sLifter->Set(DoubleSolenoid::kReverse);}
		if(arms == true) {sArm->Set(DoubleSolenoid::kForward);}
		else {sArm->Set(DoubleSolenoid::kReverse);}
		if(lever == true) {sLever->Set(DoubleSolenoid::kReverse);}
		else {sLever->Set(DoubleSolenoid::kForward);}
		if(poker == true) {sPoker->Set(DoubleSolenoid::kForward);}
		else {sPoker->Set(DoubleSolenoid::kReverse);}
		if (winchSol == true) {sWinch->Set(DoubleSolenoid::kForward);}
		else {sWinch->Set(DoubleSolenoid::kReverse);}
	}
}

void Robot::TeleopInit()
{
	//Safety feature - Turn off autoMode every time we enter Teleop so that auto is always off except when the operators have deliberately enabled it just prior to using it
	autoDistance = 0;
	autoMode = 0;
	LEnc->Reset();
	REnc->Reset();

	//Set dashboard automode to a safe value
	SmartDashboard::PutNumber("AutoDefense", 0);
	SmartDashboard::PutNumber("AutoPosition", 1);
	SmartDashboard::PutNumber("AutoMode", 0);
}

void Robot::TeleopPeriodic()
{

	checkbuttons();
	cameras->run();

	// Get joystick values
	//Axes are swapped on xbox controllers....seems weird....
	//Hopefully this is correct?????
	LaxisX = stick->GetY();
	LaxisY = stick->GetX();
	RaxisX = stick->GetRawAxis(5);
	RaxisY = stick->GetRawAxis(4);
	RTrig = stick->GetRawAxis(3);
	LTrig = stick->GetRawAxis(2);
	RaxisY *= -1;


	if (bStart == true && bStartHold == false)
	{
		direction = !direction;
	}


	if(direction == false)
	{
		LaxisX *= -1;
		LaxisY *= -1;
		RaxisX *= -1;
		//RaxisY *= -1;
	}


	//ArcadeDrive method documentation LIES.
	//Turn value is first argument, move value is 2nd argument
	drive->ArcadeDrive(LaxisX, RaxisY);
	lmotspeed = lmotor->Get();
	rmotspeed = rmotor->Get();

	if (bY == true)
	{
		winchMan = true;
	} else {winchMan = false;}

	if (bLS2 == true)
	{
		stateMan = true;
	} else {stateMan = false;}

	//Pressure Sensor Code
	Pres = PreSen->GetAverageVoltage();
	Pres = 250 * (Pres/5) - 25;

	if(Pres>=45)
	{
		pressGood = true;
	}
	else
	{
		pressGood = false;
	}


	//Range Finder Math
	//float Vm = RaFin->GetVoltage();
	//float range = (Vm*1000)*((5/4.88)*.03937);

	//Update all joystick buttons

	ArmEncValue = ArmEnc->Get();
	gyroAngle = gyro->GetAngle();

	//Get sensor inputs
	phoSensorVal = PhoSen->Get();

	//Math for winch thing
	//Combines both triggers into a single command for the winch motors
	//Positive command extends the winch, negative retracts it
	//limit winch extend power
	LTrig *= .3;
	RTrig *= -1;
	Trig = LTrig + RTrig;

	//Automatic winch control
	if (winchMan == false)
	{
		if (bRB == true) //If winch is commanded to auto-extend
		{
			winchSol = true;
			setWinch = winchMaxExtendPower; //Set winch to extend at full speed
			if (ArmEncValue >= 5000)
			{
				setWinch = 0; //When the winch hits the proper height, turn it off
			}
		}
		else { setWinch = 0;} //stop extending the winch if the operator lets go of the button

		if (bLB == true) //If winch is commanded to auto-retract
		{
			winchSol = true;
			setWinch = -0.5; //Set winch to retract at a certain power
			if (ArmEncValue <= 200)
			{
				setWinch = winchHold; //If the winch has raised the robot to a certian value, set the winch to the winch hold value and turn off the treads
				//drive->Drive(0, 0);
				lmotspeed = 0;
				rmotspeed = 0;
			}
			else if (ArmEncValue <= 500) //If the winch has raised the robot to a certian value, turn on the treads
			{
				//drive->Drive(0.5, 0);
				lmotspeed = 0.5;
				rmotspeed = 0.5;
			}
			else {lmotspeed = 0; rmotspeed = 0;}
		}
		else { setWinch = 0;} //stop retracting the winch if the operator lets go of the button
	}

	/*
	arms = part of the robot that grabs the ball
	lever = part of robot that moves the arms inside and outside the robot
	poker = part of robot on front that extends outward
	lifter = part of robot that will extend beneath the robot to lift it up

	ARMS
		true = open
		false = closed
	LEVER
		true = out of robot
		false = in robot
	POKER
		true = extended
		false = retracted
	LIFTER
		true = robot on ground
		false = robot tilted

	DOUBLE SOLENOID CLASS
		kForward = true
		kReverse = false
	*/
	if (stateMan == false)
	{
		switch (currentState)
		{
				//enter case that does nothing so we can hand control back to the
				//state machine without moving stuff
			case ENTER:
				poker = false; //Poker retracted
				if (bA2 == true && bA2Hold == false) { currentState = WT_FOR_BALL; }
				if (bRB2 == true && bRB2Hold == false) { currentState = UNLOAD; }
			break;

			case WT_FOR_BALL: //Waiting for the ball to trip the photo sensor
				//Added new exit path to allow operator to stop trying to capture a ball by releasing the A button
				if (bA2 == false) { currentState = ENTER; }

				//Open arms when entering ball capture
				arms = true;
				poker = false; //Poker retracted

				//If photo sensor is tripped, close the arms and change state to HOLD_BALL
				if (phoSensorVal == false)
				{
					arms = false; //Arms closed
					poker = false; //Poker retracted
					currentState = ENTER;
				}
				//If start button is pressed, change to idle state
				break;

				//Unload opens the arms but does not poke yet. This gives the arms ~40ms to clear the ball
			case UNLOAD:
				arms = true; //Arms open
				poker = false; //Poker stays retracted to give arms time to clear ball

				//Need to wait for the arms to clear the ball
				//This bit of code causes the state machine to spend one extra loop in this state
				//giving the arms about 40ms to open before the poker shoots the ball
				if(armClearDelay > 15)
				{
					currentState = SHOOT;
					armClearDelay = 0;
				}
				else {armClearDelay++;}
				break;

				//Take the shot! Then return to idle to wait for another ball capture
			case SHOOT:
				arms = true;
				poker = true;
				//Keep the arms open until the ball has cleared the
				currentState = ENTER;
				break;
		}
	}

	//Lever control is always manual
	if (bY2 == true && bY2Hold == false)
	{
		lever = !lever;
	}

	//driver can shoot ball at any time
	if (bB2 == true && bA2Hold == false)
	{
		poker = true;
	}

	//Manual mode controls engaged when left stick is held down
	if (stateMan == true)
	{
		//When A button is tapped, toggle the arms
		if (bA2 == true && bA2Hold == false)
		{
			arms = !arms;
		}
		//When B button is tapped, toggle the poker
		if (bB2 == true && bA2Hold == false)
		{
			poker = true;
		}
		else {poker = false;}

		//When X button is tapped, toggle the lifter
		if (bX2 == true && bX2Hold == false)
		{
			lifter = !lifter;
		}
		currentState = ENTER;
	}
	if (winchMan == true)
	{
		//When X button is pressed, keep a minimum hold power applied to the winch. Otherwise, run winch like normal
		if (bX == false) //If X button is not pressed
		{
			setWinch = Trig; //Set winch power to the trigger value
		}
		else
		{
			if (Trig < winchHold) //If the trigger value is greater then 0.05, the winch hold value, set the winch power to the triggers
			{
				setWinch = Trig; //Set the value of the winch power to the value of the triggers
			}
			else //If the trigger value is less than the hold value, set it to the hold value
			{setWinch = winchHold;}
		}
		if (bRB == true && bRBHold == false)
		{
			winchSol = !winchSol;
		}
	}

	/*
	//Creates two integers: t and Tcurve
	int t, Tcurve;
	 Multiplies trigger value by 100 to get percent
	t = Trig * 100;
	 Creates parabolic throttle curve with equation of y=0.000001x^4
	Tcurve = 0.000001 * pow(t, 4);
	 Creates linear throttle curve
	Tcurve = abs(t);
	*/

	if(lifter == true) {sLifter->Set(DoubleSolenoid::kForward);}
	else {sLifter->Set(DoubleSolenoid::kReverse);}
	if(arms == true) {sArm->Set(DoubleSolenoid::kForward);}
	else {sArm->Set(DoubleSolenoid::kReverse);}
	if(lever == true) {sLever->Set(DoubleSolenoid::kReverse);}
	else {sLever->Set(DoubleSolenoid::kForward);}
	if(poker == true) {sPoker->Set(DoubleSolenoid::kForward);}
	else {sPoker->Set(DoubleSolenoid::kReverse);}
	if (winchSol == true) {sWinch->Set(DoubleSolenoid::kForward);}
	else {sWinch->Set(DoubleSolenoid::kReverse);}
	
	//Apply all motor limits
	if(setWinch > winchMaxExtendPower) {setWinch = winchMaxExtendPower; }


	//Apply drive train mechanical compensation coeffcient
	lmotspeed *= motorCorrectionValue;
	winchmot->Set(setWinch);
	lmotor->Set(lmotspeed);
	rmotor->Set(rmotspeed);

	SmartDashboard::PutNumber("Left Motor Final Command: ", lmotor->Get());
	SmartDashboard::PutNumber("Right Motor Final Command: ", rmotor->Get());
	SmartDashboard::PutNumber("Left track distance ", LEnc->GetDistance());
	SmartDashboard::PutNumber("Right track distance ", REnc->GetDistance());
	SmartDashboard::PutNumber("Winch", setWinch);
	SmartDashboard::PutBoolean("Arms: \n", arms);
	SmartDashboard::PutBoolean("Lever: \n", lever);
	SmartDashboard::PutBoolean("Poker: \n", poker);
	SmartDashboard::PutBoolean("Lifter: \n", lifter);
	SmartDashboard::PutNumber("Gyro: \n", gyroAngle);
	SmartDashboard::PutNumber("Current State: ", currentState);
	SmartDashboard::PutNumber("Arm Encoder: ", ArmEncValue);
	SmartDashboard::PutBoolean("Winch Solenoid: ", winchSol);
	//SmartDashboard::PutNumber("Ultrasonic", range);
	SmartDashboard::PutBoolean("dirChange: ", dirChange);
	SmartDashboard::PutBoolean("Pressure is Good!", pressGood);
	SmartDashboard::PutNumber("Pressure: ", Pres);
}

void Robot::TestPeriodic()
{
}

void Robot::DisabledInit()
{
	//Set dashboard automode to a safe value
	SmartDashboard::PutNumber("AutoDefense", 0);
	SmartDashboard::PutNumber("AutoPosition", 1);
	SmartDashboard::PutNumber("AutoMode", 0);
}

void Robot::DisabledPeriodic()
{
	cameras->run();

	PresVoltage = PreSen->GetAverageVoltage();
	Pres = 250 * (PresVoltage/5) - 25;

	if(Pres>=45)
	{
		pressGood = true;
	}
	else
	{
		pressGood = false;
	}

	SmartDashboard::PutNumber("Left Motor Final Command: ", lmotor->Get());
	SmartDashboard::PutNumber("Right Motor Final Command: ", rmotor->Get());
	SmartDashboard::PutNumber("Left track distance ", LEnc->GetDistance());
	SmartDashboard::PutNumber("Right track distance ", REnc->GetDistance());
	SmartDashboard::PutNumber("Winch", setWinch);
	SmartDashboard::PutBoolean("Arms: \n", arms);
	SmartDashboard::PutBoolean("Lever: \n", lever);
	SmartDashboard::PutBoolean("Poker: \n", poker);
	SmartDashboard::PutBoolean("Lifter: \n", lifter);
	SmartDashboard::PutNumber("Gyro: \n", gyroAngle);
	SmartDashboard::PutNumber("Current State: ", currentState);
	SmartDashboard::PutNumber("Arm Encoder: ", ArmEncValue);
	SmartDashboard::PutBoolean("Winch Solenoid: ", winchSol);
	//SmartDashboard::PutNumber("Ultrasonic", range);
	SmartDashboard::PutBoolean("dirChange: ", dirChange);
	SmartDashboard::PutBoolean("Pressure is Good!", pressGood);
	SmartDashboard::PutNumber("Pressure: ", Pres);
	SmartDashboard::PutNumber("Pressure Sensor Voltage: ", PresVoltage);
}

void Robot::checkbuttons() {
	//Saves the previous value of the button
	bAHold = bA;
	bBHold = bB;
	bXHold = bX;
	bYHold = bY;
	bLBHold = bLB;
	bRBHold = bRB;
	bBackHold = bBack;
	bStartHold = bStart;
	bLSHold = bLS;
	bRSHold = bRS;

	bA2Hold = bA2;
	bB2Hold = bB2;
	bX2Hold = bX2;
	bY2Hold = bY2;
	bLB2Hold = bLB2;
	bRB2Hold = bRB2;
	bBack2Hold = bBack2;
	bStart2Hold = bStart2;
	bLS2Hold = bLS2;
	bRS2Hold = bRS2;

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
