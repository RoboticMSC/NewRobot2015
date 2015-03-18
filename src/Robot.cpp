#include "WPILib.h"
#include "Claw.h"
#include "Elevator.h"
#include "Rollers.h"
#include "Donuts.h"

#define DRIVEWHEEL_CIRCUMFERENCE 18.8495559215 //In inches, given 6-wheels
//Robot drives 18.8495559215 inches/rotation
#define DRIVEWHEEL_INCHES_PER_PULSE 0.07363107781
// (circumference/pulses per rotation)
#define DRIVEWHEEL_PULSES_PER_INCH 13.5812218105
// (256 pulses / circumference)

//Define Buttons
#define button_a  1 // 1
#define	button_b  2 // 2
#define button_x  3 // 3
#define	button_y  4 // 4
#define	button_left_trigger  5 // 5
#define	button_right_trigger  6 // 6
#define	button_back  7 // 7
#define	button_start  8 // 8

typedef enum
{
	ELEVATOR_ENCODER_A, //0
	ELEVATOR_ENCODER_B, //1
	RIGHT_ENCODER_A, //2
	RIGHT_ENCODER_B, //3
	LEFT_ENCODER_A, //4
	LEFT_ENCODER_B, //5
	//GYRO, //6
	LOWER_RIGHT_LIMIT_SWITCH, //6
	LOWER_LEFT_LIMIT_SWITCH, //7
	UPPER_LEFT_LIMIT_SWITCH, //8
	UPPER_RIGHT_LIMIT_SWITCH, //9
	GYRO //10
}DIGITAL_OUTPUT_CHANNEL;

typedef enum
{
	POINTLESS_DUMB_UNOCCUPIED_MOTOR, //0
	LEFT_ROLLER_MOTOR, //1
	LEFT_DRIVE_MOTOR, //2
	ELEVATOR_MOTOR, //3
	RIGHT_ROLLER_MOTOR, //4
	RIGHT_DRIVE_MOTOR //5
//Add your drive-train motors to this enumeration, and have joeTalon(DRIVE_TRAIN_MOTOR)
}VICTOR_CHANNEL;

typedef enum
{
	AUTONOMOUS_JUST_DRIVE,
	AUTONOMOUS_SINGLE_TOTE,
	AUTONOMOUS_THREE_TOTE_STACK,
	AUTONOMOUS_TWO_CONTAINERS,
	AUTONOMOUS_ONE_CONTAINER
}AUTONOMOUS_OPTIONS;


typedef enum
{
	SOLENOID_DRIVE_SHIFT_EXTEND, //Gear shift up
	SOLENOID_DRIVE_SHIFT_RETRACT, //Gear shift down
	SOLENOID_ELEVATOR_EXTEND,
	SOLENOID_ELEVATOR_RETRACT,
	SOLENOID_ROLLERS_EXTEND,
	SOLENOID_ROLLERS_RETRACT,
	SOLENOID_ELEVATOR_BRAKE_EXTEND, //Release brake
	SOLENOID_ELEVATOR_BRAKE_RETRACT //Engage brake

}SOLENOID_CHANNEL_MOD_0;

typedef enum
{
	SOLENOID_CLAW_EXTEND,
	SOLENOID_CLAW_RETRACT

}SOLENOID_CHANNEL_MOD_1;

class Robot: public IterativeRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
//	Joystick driver;
//	Joystick manipulator;
	//SmartDashboard driverDashboard;
	LiveWindow *lw;

	SendableChooser *autoselection;
//	VictorSP joeTalon;
	VictorSP leftdrive; //I can probably change these back
	VictorSP rightdrive;
//	VictorSP kaylaTalon;
	Solenoid shiftUpExtend;
	Solenoid shiftUpRetract;
	Encoder rightEncoder;
	Encoder leftEncoder;

	Claw claws;
	Elevator elevator;
	Compressor compressor;
	Rollers rollers;
	Gyro gyro1;

	AUTONOMOUS_OPTIONS selectedAutonomousRoutine;

	bool clawOpen = true; // Is true if the claw is open
	bool elevatorExtended = false; // Is true if the elevator is in the forward position
	bool rollersOpen = false;
	bool shiftUp = false;
	bool collision = false; // Is true of claw will crash
	bool delayedOpen = false;
	bool goingUp = false;
	bool goingDown = false;
	bool isBrakeOn = false;	//Should this start off as true or false?
	bool LevelSet = false; // Variable that is true if the PID is set to a level
	bool loading = false;
// Buttons
	bool x_pressed = false;
	bool y_pressed = false;
	bool b_pressed = false;
	bool a_pressed = false;
	bool back_pressed = false;
	bool start_pressed = false;
	bool left_trigger_pressed = false;
	bool right_trigger_pressed = false;
	double load_height = 0;
	int elevatorLevel = 0; // Defines the level of the elevator
	int elevatorLevel_prev = 0;
	int step=0; // Load function case increment
//POV
	int pov_up = 0;
	int pov_down = 180;
	int pov_left = 90;
	int pov_right = 270;

	float intakeSpeed = 0.0;
	float throttledamper = .75;
	float turningdamper = .75;
	Timer elevatorpause;
	Timer loadtimer;

//Auto Variables
	double pulse_per_inch = 72.92;
	double KP_drive;
	double KD_drive;
	double KI_drive;
	double KP_turn;
	double KD_turn;
	double KI_turn;
	double setpos;
	double setang;
	double drivepos;
	double driveang;
	double maxdrive;
	double maxturn;
	double control_dist;

	double DriveError=0;
	double DriveError_prev=0;
	double D_DriveError=0;
	double I_DriveError=0;
	double TurnError=0;
	double TurnError_prev=0;
	double D_TurnError=0;
	double I_TurnError=0;
	double drive_tol; // drive tolerance
	double turn_tol; // angle tolerance
	double drivespeed;
	double turnspeed;

	int autostep = 0; // autonomous case increment



//Unsused variables

	//Timer brakeTime;
	Timer autonTimer;
	//Timer dropRoutineTimer;
	//	SendableChooser *mendableBruiser;
//	bool activatedElevatorDown = true;
//	bool activatedElevatorUp = true;
//	bool firstTime = true;
//	bool numanumamaticIsPressed = false;
	bool droveStraight = true;
//	bool turnt = true;
//	bool yReleased;
	//bool gyroReset = false;
	int autoLoopCounter = 0;
//	int encoderCountNow = 0;
//	int encoderCountPrev = 0;

public:
	Robot() : //Define the robot
//		myRobot(joeTalon, kaylaTalon),	// these must be initialized in the same order
		myRobot(leftdrive, rightdrive),	// these must be initialized in the same order
		stick(0),// as they are declared above.
		lw(NULL),
//		mendableBruiser(),
		autoselection(),
//		joeTalon(LEFT_DRIVE_MOTOR),
//		kaylaTalon(RIGHT_DRIVE_MOTOR),
		leftdrive(LEFT_DRIVE_MOTOR),
		rightdrive(RIGHT_DRIVE_MOTOR),
		shiftUpExtend(0, SOLENOID_DRIVE_SHIFT_EXTEND),
		shiftUpRetract(0, SOLENOID_DRIVE_SHIFT_RETRACT),
		rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, true),
		leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B, true),
//		clicker(0),
		claws(SOLENOID_CLAW_EXTEND, SOLENOID_CLAW_RETRACT),
		elevator(SOLENOID_ELEVATOR_EXTEND, SOLENOID_ELEVATOR_RETRACT, SOLENOID_ELEVATOR_BRAKE_EXTEND, SOLENOID_ELEVATOR_BRAKE_RETRACT, ELEVATOR_MOTOR, ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B, LOWER_LEFT_LIMIT_SWITCH, LOWER_RIGHT_LIMIT_SWITCH, UPPER_LEFT_LIMIT_SWITCH, UPPER_RIGHT_LIMIT_SWITCH),
		compressor(5),
		rollers(SOLENOID_ROLLERS_EXTEND, SOLENOID_ROLLERS_RETRACT, RIGHT_ROLLER_MOTOR, LEFT_ROLLER_MOTOR /* , rollerSpeed */),
		gyro1(0)
		//autoLoopCounter(0),
		//lastCurve(0)

	{
	//	myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
		SmartDashboard::init();
		SmartDashboard::PutBoolean("High gear on?", shiftUpExtend.Get());
	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		//vvv This is the code for changing the autonomous mode through SmartDashboard
//		mendableBruiser = new SendableChooser();
//		mendableBruiser->AddDefault("Drive Forward One Foot", (void *) AUTONOMOUS_JUST_DRIVE);
//		mendableBruiser->AddObject("Single Tote", (void *)(AUTONOMOUS_SINGLE_TOTE));
//		mendableBruiser->AddObject("Three Tote Stack", (void *)(AUTONOMOUS_THREE_TOTE_STACK));
//		mendableBruiser->AddObject("Two Containers", (void *)(AUTONOMOUS_TWO_CONTAINERS));
//		mendableBruiser->AddObject("One Container Into Auto Zone?", (void *)(AUTONOMOUS_ONE_CONTAINER));
//		SmartDashboard::PutData("Autonomous Modes", mendableBruiser);
		//^^^

		//vvv This is the code for changing the autonomous mode through SmartDashboard
		autoselection = new SendableChooser();
		autoselection->AddDefault("Drive Forward One Foot", (void *) AUTONOMOUS_JUST_DRIVE);
		autoselection->AddObject("Single Tote", (void *)(AUTONOMOUS_SINGLE_TOTE));
		autoselection->AddObject("Three Tote Stack", (void *)(AUTONOMOUS_THREE_TOTE_STACK));
		autoselection->AddObject("Two Containers", (void *)(AUTONOMOUS_TWO_CONTAINERS));
		autoselection->AddObject("One Container Into Auto Zone?", (void *)(AUTONOMOUS_ONE_CONTAINER));
		SmartDashboard::PutData("Autonomous Modes", autoselection);
		//^^^

		SmartDashboard::PutNumber("Elevator_P",0.1);
		SmartDashboard::PutNumber("Elevator_I",0.0);
		SmartDashboard::PutNumber("Elevator_D",0.0);
		SmartDashboard::PutNumber("Throttle Dampening", 0.5);
		SmartDashboard::PutNumber("Steering Dampening", 0.5);
	}

	void AutonomousInit()
	{
		//vvv Get selected autonomous from the SmartDashBoard
//		selectedAutonomousRoutine = (AUTONOMOUS_OPTIONS)((int) mendableBruiser->GetSelected());
	//	selectedAutonomousRoutine = (AUTONOMOUS_OPTIONS)((int) autoselection->GetSelected());

		autonTimer.Reset();
		autonTimer.Start();
		//autoLoopCounter = 0;
		shiftUpExtend.Set(true);
		shiftUpRetract.Set(false);
		compressor.Start();

		rightEncoder.Reset();
		leftEncoder.Reset();
		gyro1.Reset();
	}

	void AutonomousPeriodic()
	{
		//SmartDashboard::PutNumber("AutoTime", autonTimer.Get());
		//AutoDriveForwardTime();
		//AutoDriveCan();
		//AutoToteStack();
		AutoTestPID();
		SmartDashboard::PutNumber("Right Drive Encoder", rightEncoder.Get());
		SmartDashboard::PutNumber("Left Drive Encoder", leftEncoder.Get());
		SmartDashboard::PutNumber("Gyro Angle", gyro1.GetAngle());
	/*
		elevator.Execute();
		SmartDashboard::PutString("Choose an Autonomous!", "autonOptions");

		//TODO: Is this it?!?!: switch (autonomousCommand)
		//(Command *) autonomousCommand
		switch (selectedAutonomousRoutine)
		{
		case AUTONOMOUS_JUST_DRIVE:
			AutonJustDrive();
			//AutoDriveForwardTime();
			break;
		case AUTONOMOUS_SINGLE_TOTE:
			AutonSingleTote();
			break;
		case AUTONOMOUS_THREE_TOTE_STACK:
			AutonThreeTote();
			break;
		case AUTONOMOUS_TWO_CONTAINERS:
			AutonTwoContainers();
			break;
		case AUTONOMOUS_ONE_CONTAINER:
			AutonOneContainer();
			break;
		}
		*/
	}

	void TeleopInit()
	{
		compressor.Start();
		shiftUpRetract.Set(true);
		shiftUpExtend.Set(false);
		elevatorpause.Reset();
		loadtimer.Reset();
		elevator.Reset();
//		elevator.RetractElevator();
		elevatorExtended = false;
		rightEncoder.Reset();
		leftEncoder.Reset();
	}

	void TeleopDisabled()
	{
		//driverDashboard.PutBoolean("Extended", extended);
		elevator.Execute();
	}

	void TeleopPeriodic()
	{
		elevator.Execute();
		SmartDashboard::PutBoolean("High gear ON?", shiftUpExtend.Get());
		SmartDashboard::PutBoolean("OverRide", collision);
		SmartDashboard::PutString("Do it work?!", "Aw yeah");
		SmartDashboard::PutNumber("Right Drive Encoder", rightEncoder.Get());
		SmartDashboard::PutNumber("Left Drive Encoder", leftEncoder.Get());
//		SmartDashboard::PutNumber("Left Side Speed", leftdrive.Get());
//		SmartDashboard::PutNumber("Right Side Speed", rightdrive.Get());

		//double rightEncoderRate = rightEncoder.GetRate();
		//double rightRPM = (rightEncoderRate/256) * 60;

		//vvv SmartDashboard motor buffer setter

		//PID test helper concept

		//Drop Routine Start
//		float p = SmartDashboard::GetNumber("Elevator_P");
//		float i = SmartDashboard::GetNumber("Elevator_I");
//		float d = SmartDashboard::GetNumber("Elevator_D");

//		static bool wasButton8Pressed = false;//static means that the variable is saved even after you leave the function
//		if (stick.GetRawButton(8) == true)
//		{
//
//			if (wasButton8Pressed == false)
//			{
//				elevator.SetPID(p,i,d);
//				wasButton8Pressed = true;
//			}
//		}
//		else
//		{
//			wasButton8Pressed = false;
//		}



// Drivetrain motor values

		if ((stick.GetRawAxis(4) < -0.04 || stick.GetRawAxis(4) > 0.04) || (stick.GetRawAxis(1) < -0.04 || stick.GetRawAxis(1) > 0.04))
		{
			//myRobot.ArcadeDrive((stick.GetRawAxis(4) * turningdamper), (stick.GetRawAxis(1) * throttledamper)); //0.7 dampens the steering sensitivity, modify to taste
			myRobot.ArcadeDrive((stick.GetRawAxis(1) * throttledamper),(stick.GetRawAxis(4) * turningdamper)); //0.7 dampens the steering sensitivity, modify to taste

		}
		else
		{
			myRobot.ArcadeDrive(0.0,0.0);
		}

// Calibrate elevator or cancel target position
		if (stick.GetRawButton(button_start) == true || start_pressed==true) //Calibrate Elevator
		{
			if (elevator.Calibrate()==true) //Runs the "Calibrate" function and returns true when complete
			{
				start_pressed=false;
				elevatorLevel = 0; // Set elevator target level to zero
			}
			else
			{
				LevelSet=false; // Disable PID
				start_pressed=true;
			}
		}

		if (stick.GetRawButton(button_back) == true) //Cancels set point
		{
			LevelSet=false; // Disable PID
			loading == false;
		}

// Check to see if collision case is true
		if (elevator.IsCrashing() == true || elevatorExtended == true)
		{
			collision = true; // The elevator and rollers could collide
		}
		else if (elevator.IsCrashing() == false)
		{
			collision = false; // The elevator and rollers will not collide
		}

// Open the claw
		if (stick.GetRawButton(button_right_trigger) == true /*&& clawOpen == false*/ && loading == false) //Open claw only if POV_right is held down
		{
			claws.OpenClaw();
		//	clawOpen = true;
		}

		else if (loading == false) // Normally closed
		{
		//	clawOpen = false;
			claws.CloseClaw();
		}

// Loading Operation
		SmartDashboard::PutNumber("loadtimer", loadtimer.Get());
		SmartDashboard::PutNumber("Step", step);
		SmartDashboard::PutBoolean("loadingvar", loading);
		SmartDashboard::PutNumber("Elevator Level", elevatorLevel);
		if ((stick.GetPOV(0) == pov_left || loading == true) && (elevatorLevel >=2 || step==3 || step==4))
		{
			loadtote();
			/*
			loading = true;

			switch (step)
			{
			case 0:
				elevatorLevel = 8;
				LevelSet = true;
				load_height = elevator.LevelHeight(elevatorLevel);
				if (elevator.AtPosition(load_height) == false) {
					//wait till position
				}

				else
				{
					step = 1;
				}
			break;

			case 1:
				if (loadtimer.Get()<=0)
				{
					loadtimer.Start();
				}

				else if (loadtimer.Get()>0.0 && loadtimer.Get()<1)
				{
					// Wait for time
					claws.OpenClaw();
				}
				else {
					loadtimer.Stop();
					loadtimer.Reset();

					step = 2;
				}
			break;

			case 2:
				elevatorLevel = 0;
				LevelSet = true;
				step = 3;
			break;

			case 3:
				if (elevator.BottomlimitHit()==true)
				{
					claws.CloseClaw();
					loading = false;
					step =0;
				}

			break;
			}

			*/
		}

//Extend Elevator
		if (stick.GetPOV(0) == pov_up && elevatorExtended == false)
		{
			elevator.ExtendElevator();
			elevatorExtended = true;
		}

//Retract Elevator
		if (stick.GetPOV(0) == pov_down && elevatorExtended == true)
		{
			elevator.RetractElevator();
			elevatorExtended = false;
		}

// Set elevator level setpoint

		// Increment Elevator by 1
		if (stick.GetRawButton(button_x) == true && x_pressed == false && loading == false)
		{
			x_pressed = true; //set button status to "pressed"
		}

		else if (x_pressed == true && stick.GetRawButton(button_x) == false && loading == false) // If the elevator button was pressed and released
		{
			//Increment the elevator level
			if(elevatorLevel < elevator.maxLevel()) // Checks if the elevator is at the maximum level
			{
				elevatorLevel = elevatorLevel + 1; // Increment
				// setLevel0 = true;
				LevelSet = true;
			}
			else // At maximum level, do not increment elevator
			{
				//do not increment the elevator level
			}

			x_pressed = false; //reset button status to "released"
		}
		// Decrement Elevator by 1
		if (stick.GetRawButton(button_a) == true && a_pressed == false && loading == false)
		{
			a_pressed = true; //set button status to "pressed"
		}

		else if (a_pressed == true && stick.GetRawButton(button_a) == false && loading == false) // If the elevator button was pressed and released
		{
			//Decrement the elevator level
			if(elevatorLevel > elevator.minLevel()) // Checks if the elevator is at the minimum level
			{
				elevatorLevel = elevatorLevel - 1; // Decrement
				LevelSet = true;
			}
			else // At minimum level, do not decrement elevator
			{
				//do not decrement the elevator level
			}

			a_pressed = false; //reset button status to "released"
		}
		// Set to bottom level
		if (stick.GetRawButton(button_b) == true && b_pressed == false && loading == false)
		{
			b_pressed = true; //set button status to "pressed"
		}

		else if (b_pressed == true && stick.GetRawButton(button_b) == false && loading == false) // If the elevator button was pressed and released
		{
			//Set elevator level to minimum elevator level
			elevatorLevel = elevator.minLevel();
			LevelSet = true;
			b_pressed = false; //reset button status to "released"
		}

		// Set to top level
		if (stick.GetRawButton(button_y) == true && y_pressed == false && loading == false)
		{
			y_pressed = true; //set button status to "pressed"
		}

		else if (y_pressed == true && stick.GetRawButton(button_y) == false && loading == false) // If the elevator button was pressed and released
		{

			//Set elevator level to minimum elevator level
			elevatorLevel = elevator.maxLevel();
			LevelSet = true;
			y_pressed = false; //reset button status to "released"
		}

		//These can be removed
		SmartDashboard::PutNumber(" my inc", elevatorLevel);
		SmartDashboard::PutBoolean("Level Set Bool", LevelSet);
		SmartDashboard::PutNumber("Pause Time", elevatorpause.Get());

		SmartDashboard::PutBoolean("Collision", collision);
		SmartDashboard::PutBoolean("Delay Open", delayedOpen);
		SmartDashboard::PutNumber(" Prev level", elevatorLevel_prev);
		// Set PID setPoint
		if (LevelSet == true) // LevelSet is a bool that is true if the PID is set at a level and false if the PID is not set at a level
		{

			if(elevatorLevel <2) // If the elevator is at a level less than 2, then pause the elevator to allow time for the rollers to open
			{
				delayedOpen = true;
				if(elevatorpause.Get()<=0.0)
				{
					elevatorpause.Reset();
					elevatorpause.Start();
				}
				else if (elevatorpause.Get()<0.2 && elevatorpause.Get()>0.0)
				{

				}
				else
				{

					LevelSet = elevator.setPID(.6,-.4, elevatorLevel);
				}
			}
			else
			{
				delayedOpen = false;
				elevatorpause.Reset();
				LevelSet = elevator.setPID(.6,-.4, elevatorLevel); //SetPID(max_out,min_out, elevatorLevel) where elevatorLevel is an integer corresponding to preset locations in the switch case. SetPID will return true or false to enable or disable the elevator PID
			}
		}

//		if (LevelSet == true) // LevelSet is a bool that is true if the PID is set at a level and false if the PID is not set at a level
//		{
//				LevelSet = elevator.setPID(.6,-.4, elevatorLevel); //SetPID(max_out,min_out, elevatorLevel) where elevatorLevel is an integer corresponding to preset locations in the switch case. SetPID will return true or false to enable or disable the elevator PID
//		}

// Drivetrain shifting. Change this to shift only when the button is held

		//Shift Up Gear when left trigger is held
		if (stick.GetRawButton(button_left_trigger) == true)
		{

			shiftUpExtend.Set(false);
			shiftUpRetract.Set(true);
		}

		//Shift Down Gear Normally
		else
		{
			shiftUpExtend.Set(true);
			shiftUpRetract.Set(false);
		}

//Control Rollers
		/* Old roller control
		//Spin Rollers In
		if (stick.GetRawAxis(2) >= 0.1  && stick.GetRawAxis(3) == 0.0 && collision == false) // Those triggers are probably variable
		{
			intakeSpeed = 1.0;
			rollers.Barf(intakeSpeed);
			rollers.CloseRollers();
		}

		//Spin Rollers Out
		else if (stick.GetRawAxis(3) >= 0.1 && stick.GetRawAxis(2) == 0 && collision == false)
		{
			intakeSpeed = 1.0;
			rollers.Eat(intakeSpeed);
			rollers.CloseRollers();
		}

		//Stop Rollers
		else
		{
			rollers.OpenRollers();
			rollers.RollersIdle();
		}
		*/

		// New roller control

		//Eject
		if (stick.GetRawAxis(2) >= 0.1  && stick.GetRawAxis(3) == 0.0 && collision == false) // If left trigger is pressed close claw and eject
		{

//			intakeSpeed = 1.0;
//			rollers.Barf(intakeSpeed);
//			rollers.CloseRollers();
//			rollersOpen = false;
			rollers.OpenRollers();
			rollers.RollersIdle();
			rollersOpen = true;
		}

		//Intake
		else if (stick.GetRawAxis(3) >= 0.1 && stick.GetRawAxis(2) == 0 && collision == false && loading == false) // If right trigger is pressed open claw and idle
		{
			intakeSpeed = 1.0;
			rollers.Eat(intakeSpeed);
			rollersOpen = false;
		}

		//Normal configuration when not colliding
		else if (collision == false && delayedOpen == false && loading == false) // If no triggers are pressed
		{
			//intakeSpeed = 1.0;
			//rollers.Eat(intakeSpeed);
			rollers.CloseRollers();
			rollers.RollersIdle();
			rollersOpen = false;
		}

		//Normal configuration if colliding
		else if (collision == true || delayedOpen == true)
		{
			rollers.OpenRollers();
			rollers.RollersIdle();
			rollersOpen = true;
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void loadtote()
	{
		loading = true;

		switch (step)
		{
		case 0:
			elevatorLevel = 8;
			LevelSet = true;
			load_height = elevator.LevelHeight(elevatorLevel);
			if (elevator.AtPosition(load_height) == false) {
				//wait till position
			}

			else
			{
				step = 1;
			}
		break;

		case 1:
			if (loadtimer.Get()<=0)
			{
				loadtimer.Start();
			}

			else if (loadtimer.Get()>0.0 && loadtimer.Get()<1)
			{
				// Wait for time
				claws.OpenClaw();
			}
			else {
				loadtimer.Stop();
				loadtimer.Reset();

				step = 2;
			}
		break;

		case 2:
			loadtimer.Stop();
			loadtimer.Reset();
			loadtimer.Start();
			elevatorLevel = 0;
			LevelSet = true;
			step = 3;
		break;

		case 3:
			if (elevator.BottomlimitHit()==true)
			{
				claws.CloseClaw();
				//loading = false;
				loading = true;
				//step =0;
				loadtimer.Stop();
				loadtimer.Reset();
				loadtimer.Start();

				step = 4;
			}
			else if (elevator.BottomlimitHit()==false && loadtimer.Get()>1.5)
			{
				loading = false;
				LevelSet = true;
				loadtimer.Stop();
				loadtimer.Reset();
				step = 0;
			}

		break;

		case 4:
			{

				if (loadtimer.Get()>=0.0 && loadtimer.Get()<.3)
				{
					//wait
				}
				else{
				elevatorLevel = 1;
				LevelSet = true;
				loading = false;
				loadtimer.Stop();
				loadtimer.Reset();
				step = 0;
				}
			}
		break;
		}

	}


// Aaron's Auto Functions
	void AutoDriveForwardTime()
	{
		if (autonTimer.Get() <=2.3)
		{
			leftdrive.Set(-0.40);
			rightdrive.Set(0.40);
		}
		else if (autonTimer.Get()>2.3 && autonTimer.Get()<=3)
		{
			leftdrive.Set(-0.8);
			rightdrive.Set(-0.8);
		}
		else
		{
			leftdrive.Set(0.0);
			rightdrive.Set(0.0);
			autonTimer.Stop();
		}
	}

	void AutoDriveCan()
	{
		rollers.OpenRollers();
		rollers.RollersIdle();
		if (autonTimer.Get()<= 1.0)
		{
			elevator.setPID(.6,-.4, 2);	// Set level 3
			claws.OpenClaw();
			clawOpen = true;
		}

		else if (autonTimer.Get() > 1.0 && autonTimer.Get()<=2) // Wait for 0.3 seconds before starting this if statement
		{
			elevator.ExtendElevator();
			elevatorExtended = true;

			leftdrive.Set(-0.10);
			rightdrive.Set(0.10);
		}
		else if (autonTimer.Get() >2 && autonTimer.Get()<=2.5)
		{
			elevator.setPID(.6,-.4, 3);	// Set level 3
		}
		else if (autonTimer.Get() >2.5 && autonTimer.Get()<=3)
		{
			elevator.setPID(.6,-.4, 3);	// Set level 3
			claws.CloseClaw();
			clawOpen = false;
		}
		else if (autonTimer.Get()>3 && autonTimer.Get()<=4)
		{
			elevator.setPID(.6,-.4, 3);	// Set level 3
			leftdrive.Set(-0.8);
			rightdrive.Set(-0.8);
		}
		else if (autonTimer.Get()>4 && autonTimer.Get()<=4.5)
		{
			elevator.setPID(.6,-.4, 2);	// Set level 2
			leftdrive.Set(0.0);
			rightdrive.Set(0.0);
		}
		else if (autonTimer.Get()>5 && autonTimer.Get()<=6)
		{
			elevator.setPID(.6,-.4, 0);	// Set level 0
			claws.OpenClaw();
			clawOpen = true;
		}
		else
		{
			leftdrive.Set(0.0);
			rightdrive.Set(0.0);
			autonTimer.Stop();
		}
	}

	void AutoToteStack()
	{


		switch (autostep)
		{
			case 0: // Drive forward to first tote
				if ((abs(rightEncoder.Get())+abs(leftEncoder.Get()))/2 <= 4*pulse_per_inch)
				{
					rollers.Eat(1.0);
					rollers.CloseRollers();
					leftdrive.Set(-0.3);
					rightdrive.Set(0.3);
				}
				else
				{

					autonTimer.Reset();
					autonTimer.Start();
					autostep=1;
				}

			break;

			case 1: // load tote
				if (autonTimer.Get() <= 0.5) //Wait for 1 sec to simulate load function
				{
					rollers.Eat(1.0);
					rollers.CloseRollers();
					leftdrive.Set(0.0);
					rightdrive.Set(0.0);
				}
				else
				{
					autostep=2;
				}

			break;

			case 2: // load tote
				if (autonTimer.Get() <= 4.0) //Wait for 1 sec to simulate load function
				{
					rollers.Eat(0.0);
					rollers.CloseRollers();
					leftdrive.Set(0.0);
					rightdrive.Set(0.0);
				}
				else
				{
					autostep=3;
				}

			break;
			case 3: // Turn slightly around can
				if (gyro1.GetAngle()<30)
				{
					leftdrive.Set(-0.4);
					rightdrive.Set(-0.4);
				}
				else
				{

					leftdrive.Set(0.0);
					rightdrive.Set(0.0);
					rightEncoder.Reset();
					leftEncoder.Reset();
					autostep = 4;
				}
			break;

			case 4: // Drive forward slightly
				if ((abs(rightEncoder.Get())+abs(leftEncoder.Get()))/2 <= 10*pulse_per_inch)
				{
					leftdrive.Set(-0.3);
					rightdrive.Set(0.3);
				}
				else
				{
					autostep = 5;
				}
			break;

			case 5: // Turn slightly around can
				if (gyro1.GetAngle()>=0)
				{
					leftdrive.Set(0.4);
					rightdrive.Set(0.4);
				}
				else
				{

					leftdrive.Set(0.0);
					rightdrive.Set(0.0);
					rightEncoder.Reset();
					leftEncoder.Reset();
					autostep = 6;
				}

		}
	}

	void AutoTestPID()
	{
		DrivePID(12*pulse_per_inch,rightEncoder.Get());
	}

	void DrivePID (double setpos,double drivepos) // setpos and position given in encoder counts
	{
	maxdrive = 0.4;
	KP_drive=.5;
	KD_drive = .2;
	KI_drive = 0;
	control_dist = 12*pulse_per_inch; // The control distance is the distance at which the PID take effect. Beyond this distance, the drive train moves at a defined speed

	DriveError = (setpos-drivepos)/control_dist; // normalized DriveError with respect to the control distance
	D_DriveError = DriveError-DriveError_prev;
	I_DriveError = I_DriveError+DriveError;


	drivespeed = KP_drive*DriveError+KD_drive*D_DriveError+KI_drive*I_DriveError;

	DriveError_prev = DriveError;

		if (abs(setpos-drivepos)> control_dist && AtDrivePos(setpos))
		{
			leftdrive.Set(-maxdrive);
			rightdrive.Set(maxdrive);
		}
		else
		{
			leftdrive.Set(-drivespeed);
			rightdrive.Set(drivespeed);
		}

	}

	void TurnPID(double setang,double driveang)
	{
	maxturn = 1;
	KP_turn=.5;
	KD_turn = .2;
	KI_turn = 0;

	TurnError = (setang-driveang)/360; // normalized TurnError
	D_TurnError = TurnError-TurnError_prev;
	I_TurnError = I_TurnError+TurnError;

	turnspeed = KP_turn*TurnError+KD_turn*D_TurnError+KI_turn*I_TurnError;

	TurnError_prev=TurnError;

	// set motors to be turnspeed
	}

	void ResetDrivePID()
	{
		DriveError=0.0;
		DriveError_prev=0.0;
		D_DriveError=0;
		I_DriveError=0;
	}

	void ResetTurnPID()
	{
		TurnError=0.0;
		TurnError_prev=0.0;
		D_TurnError=0;
		I_TurnError=0;
	}

	bool AtDrivePos(double setpos)
	{
		drive_tol = 10;
		if( abs((abs(rightEncoder.Get())+abs(leftEncoder.Get()))/2.0-setpos) <=drive_tol)
		{
			return true;
		}
		else
		{
			return false;
		}

	}

	bool AtTurnAng(double setang)
	{
		turn_tol = 1.0;
		if( abs((abs(rightEncoder.Get())+abs(leftEncoder.Get()))/2.0-setang) <= turn_tol)
		{
			return true;
		}
		else
		{
			return false;
		}

	}
	//Autonomous functions zone start

//	void Auto_DriveStraightForwardDistance(float distance, float speed) //distance in inches
//	{
////Clean up this AUTO
//		//JOE: You can't reset these here, or you'll always Get() zero
//		rightEncoder.Reset();
//		leftEncoder.Reset();
////		if(droveStraight == true)
////		{
////			rightEncoder.Reset();
////			leftEncoder.Reset();
////			droveStraight = false;
////		}
//
//		//JOE: DO this rightEncoder.SetDistancePerPulse(or whatever that function is), then here, use rightEncoder.GetDistance
//		if (rightEncoder.Get() < (distance * DRIVEWHEEL_PULSES_PER_INCH) && leftEncoder.Get() < (distance * DRIVEWHEEL_PULSES_PER_INCH)) //Might need a buffer here
//		{
//			myRobot.Drive(speed, 0.0);
//		}
//
//		else
//		{
//			myRobot.Drive(0.0, 0.0);
//			droveStraight = true;
//			return;
//		}
//	}
//
//	void Auto_DriveStraightBackwardDistance(float distance, float speed) //distance in inches
//		{
//			rightEncoder.Reset();
//			leftEncoder.Reset();
//	//		if(droveStraight == true)
//	//		{
//	//			rightEncoder.Reset();
//	//			leftEncoder.Reset();
//	//			droveStraight = false;
//	//		}
//			if (rightEncoder.Get() > (distance * DRIVEWHEEL_PULSES_PER_INCH) && leftEncoder.Get() > (distance * DRIVEWHEEL_PULSES_PER_INCH)) //Might need a buffer here
//			{
//				myRobot.Drive(-speed, 0.0);
//			}
//			else
//			{
//				myRobot.Drive(0.0, 0.0);
//				droveStraight = true;
//				return;
//			}
//		}
//
//	void Auto_ZeroPointTurn(float degrees, float speed)
//	{
//		//TODO: figure out rotations/degree for the drive wheels when both sides are moving opposite directions (i.e. stationary turning)
//		//TODO edit: I do believe that we can do this with gyro code which is pretty simple.
//
//		gyro1.Reset();
//
//		if (gyro1.GetAngle() != degrees || gyro1.GetAngle() != (degrees + 1) ||gyro1.GetAngle() != (degrees - 1))
//		{
//			myRobot.Drive(0.0, speed);
//		}
//		else
//		{
//			myRobot.Drive(0.0, 0.0);
//			return;
//		}
//
//		/**
//		if (turnt == true)
//		{
//			rightEncoder.Reset();
//			leftEncoder.Reset();
//			turnt = false;
//		}
//		if (encoder angle check stuff)
//		{
//			myRobot.ArcadeDrive(0, angle conversion stuff);
//		}
//		else
//		{
//			myRobot.ArcadeDrive(0.0, 0.0);
//			turnt = true;
//		}
//		*/
//	}
//
////Autonomous functions zone end

//	void AutonJustDrive() //Drives forward. Could push tote/container
//	{
//		if (autonTimer.Get() < 10) //10 seconds is more than enough. We just don't yet know how long it will take for the robot to drive however far forward. This is how we're going to sequence autonomous stuff: giving functions set completion times. -Ben
//		{
//			Auto_DriveStraightForwardDistance(12, 0.5); //Forward one foot, for now
//		}
//	}
//
//	void AutonSingleTote() //Grasps a yellow tote, turns, drives into the Auto Zone, and goes and sets it on the Landmark
//	{
//		//For now, we'll code this in regards to starting at the middle yellow tote
//		switch (autoLoopCounter)
//		{
//
//		case 0:
//			if (autonTimer.Get() < 2)
//			{
//				if (clawOpen == true)
//				{
//					claws.CloseClaw();
//					clawOpen = false;
//
//					elevator.BrakeOff();
//					//elevator.SetLevel(4);
//				}
//
////				if (elevator.IsAtLevel() == true)
////				{
////					elevator.BrakeOn();
////				}
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//	//			Do we need to use implement something like this Joe Code vvv for anything?
//	//            if (autonReset)
//	//            {
//	//                autonDrivingForward.Reset();
//	//                autonDrivingForward.Start();
//	//            }
//	//            myRobot.Drive(0.0,0.0);
//	//            if (autonDrivingForward.Get() >=  0.2)
//	//            {
//	//                autonReset = true;
//	//                autonStepCount++;
//	//            }
//		case 1:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_ZeroPointTurn(90, 0.6);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//
//		case 2:
//			if (autonTimer.Get() < 4)
//			{
//				Auto_DriveStraightForwardDistance(107, 1.0);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 3:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_ZeroPointTurn(-90, 0.6);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 4:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(24, 1.0);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 5:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOff();
//			//	elevator.SetLevel(0);
////				if (elevator.IsAtLevel() == true)
////				{
////					elevator.BrakeOn();
////					claws.OpenClaw();
////					clawOpen = true;
////				}
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 6:
//			if (autonTimer.Get() < 3)
//			{
//				Auto_DriveStraightBackwardDistance(36, 1.0);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 7:
//			myRobot.Drive(0, 0);
//			break;
//		}
//	}
//
//	void AutonThreeTote()  //Three tote stack autonomous
//	{
//
//		float gyro_angle = gyro1.GetAngle();
//		switch (autoLoopCounter)
//		{
//		case 0:
//			if (autonTimer.Get() < 2)
//			{
//				rollers.CloseRollers();
//				rollers.Eat(0.7);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 1:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOff();
//			//	elevator.SetLevel(0);
//				rollers.OpenRollers();
//				rollers.RollersIdle();
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 2:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOn();
//			//	elevator.SetLevel(2);
//				rollers.PushLeft(0.7);
//				Auto_DriveStraightForwardDistance(6, 0.3);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 3:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(57, 0.8);
//				myRobot.Drive(0.8, 0.0);
//				rollers.Eat(0.7);
//			}
//			else
//			{
//				Auto_DriveStraightForwardDistance(0, 0.0);
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 4:
//			if (autonTimer.Get() < 2)
//			{
//				myRobot.Drive(0, 0);
//				rollers.CloseRollers();
//				rollers.Eat(0.5);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 5:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOff();
//				//elevator.SetLevel(0);
//				rollers.OpenRollers();
//				rollers.RollersIdle();
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 6:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOn();
//			//	elevator.SetLevel(2);
//				rollers.PushLeft(0.7);
//				Auto_DriveStraightForwardDistance(6, 0.3);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 7:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(57, 0.8);
//				rollers.Eat(0.7);
//			}
//			else
//			{
//				Auto_DriveStraightForwardDistance(0, 0.0);
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 8:
//			if (autonTimer.Get() < 2)
//			{
//				myRobot.Drive(0, 0);
//				rollers.CloseRollers();
//				rollers.Eat(0.5);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 9:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOff();
//			//	elevator.SetLevel(0);
//				rollers.OpenRollers();
//				rollers.RollersIdle();
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 10:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOn();
//			//	elevator.SetLevel(1);
//				Auto_ZeroPointTurn(90, 0.5);
////				if (gyro_angle < 90 || gyro_angle < -90)
////				{
////					myRobot.Drive(0.0, 0.5);
////				}
////				else if (gyro_angle == 90 || gyro_angle == -90)
////				{
////					myRobot.Drive(0.0, 0.0);
////				}
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 11:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(65, 0.8);
//			}
//			else
//			{
//				Auto_DriveStraightForwardDistance(0, 0.0);
//				autoLoopCounter++;
//				autonTimer.Reset();
//
//			}
//			break;
//		case 12:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_ZeroPointTurn(90, 0.5);
////				if (gyro_angle < 180 || gyro_angle < -180)
////				{
////					myRobot.Drive(0.0, 0.5);
////				}
////				else if (gyro_angle == 180 || gyro_angle == -180)
////				{
////					myRobot.Drive(0.0, 0.0);
////				}
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 13:
//			if (autonTimer.Get() < 2)
//			{
//				elevator.BrakeOff();
//				rollers.OpenRollers();
//			//	elevator.SetLevel(0);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 14:
//			if (autonTimer.Get() < 2)
//			{
//				rollers.CloseRollers();
//				Auto_DriveStraightBackwardDistance(57, 0.8);
//				rollers.Barf(0.7);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		}
//	}
//
//	void AutonTwoContainers() //Grabs 2 containers off the step TODO: need to figure out how hooks work so we can program them
//	{
//		switch (autoLoopCounter)
//		{
//		case 0:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(48, 0.5);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 1:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(0, 0.0);
//				// do hooky stuff
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 2:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightBackwardDistance(68, 0.5);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		}
//	}
//
//	void AutonOneContainer() //Grab 1 container and goes into auto zone
//	{
//		switch (autoLoopCounter)
//		{
//		case 0:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(6, 0.5);
//				rollers.CloseRollers();
//				rollers.Eat(1.0);
//			}
//
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 2:
//			if (autonTimer.Get() < 2)
//			{
//			//	elevator.SetLevel(1);
//				rollers.OpenRollers();
//				rollers.Eat(0.0);
//			}
//			else
//			{
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//		case 3:
//			if (autonTimer.Get() < 2)
//			{
//				Auto_DriveStraightForwardDistance(50, 0.5);
//
//			}
//			else
//			{
//				Auto_DriveStraightForwardDistance(0, 0.0);
//				autoLoopCounter++;
//				autonTimer.Reset();
//			}
//			break;
//
//		}
//	}
//

};

START_ROBOT_CLASS(Robot);
