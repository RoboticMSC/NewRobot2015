/*
 /*
 * Elevator.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: Bill
 */
#include "Elevator.h"

#define LIMIT_SWITCHES_ARE_MISSING
#define PULSES_PER_REVOLUTION 250
#define PITCH_DIAMETER 2.638F
#define PI 3.1415926F
#define DISTANCE_PER_PULSE ((2*PI*PITCH_DIAMETER) / PULSES_PER_REVOLUTION)
//Pitch Diameter 2.638

Elevator::Elevator(int elevator_extend, int elevator_retract, int elevator_brake_extend, int elevator_brake_retract, int elevator_motor, int elevator_encoder_A, int elevator_encoder_B, int lower_left_limit, int lower_right_limit, int upper_left_limit, int upper_right_limit):
elevatorExtend(0, elevator_extend),
elevatorRetract(0, elevator_retract),
elevatorBrakeExtend(0, elevator_brake_extend),
elevatorBrakeRetract(0, elevator_brake_retract),
elevatorMotor(elevator_motor),
elevatorEncoder(elevator_encoder_A, elevator_encoder_B, true),
leftLowerLimit(lower_left_limit),
rightLowerLimit(lower_right_limit),
leftUpperLimit(upper_left_limit),
rightUpperLimit(upper_right_limit),
destinationLevel(LEVEL_ONE),
destinationFloor(1),
isCountSet(false),
isStopChecked(false),
elevatorPid(0.1, 0000.0000, 0.0, &elevatorEncoder, &elevatorMotor) //you must use a split pwm to drive both victors from one pwm output; then you just have an elevatorMotor victor declaration, which drives two motors
{
	elevatorEncoder.SetDistancePerPulse(DISTANCE_PER_PULSE);
//	elevatorEncoder.SetDistancePerPulse(0.044007429891485);
	elevatorEncoder.SetPIDSourceParameter(PIDSource::kDistance);
//	elevatorEncoder.SetDistancePerPulse((distPerPulse/pulsesPerRotation)) 256 pulses per rotation; ??? distance per rotation (compute this from gear ratios and
// pd = 1.751, ratio = 1:2, 2(pi)1.751
	//
	SmartDashboard::init();
	//SmartDashboard::PutNumber("Current Val", elevatorEncoder.GetDistance());
//	SmartDashboard::PutNumber("Current Encoder Pos", elevatorEncoder.Get());
	SmartDashboard::PutNumber("Distance Per Pulse", DISTANCE_PER_PULSE);

	elevatorPid.SetOutputRange(-0.2, 0.2);
	elevatorPid.SetAbsoluteTolerance(0.25);

}

void Elevator::ExtendElevator()
{
	elevatorExtend.Set(true);
	elevatorRetract.Set(false);
}

void Elevator::RetractElevator()
{
	elevatorExtend.Set(false);
	elevatorRetract.Set(true);
}

void Elevator::BrakeOn()
{
	elevatorBrakeExtend.Set(false);
	elevatorBrakeRetract.Set(true);
}

void Elevator::BrakeOff()
{
	elevatorBrakeExtend.Set(true);
	elevatorBrakeRetract.Set(false);
}

bool Elevator::IsCrashing()
{
	if (elevatorEncoder.Get() < 140 || elevatorExtend.Get() == true)
	{
	//	return true; // trick the robot to think the elevator is never crashing for testing purposes.
		return false;
	}
	else if (elevatorEncoder.Get() >= 140 && elevatorExtend.Get() == false)
	{
		return false;
	}
	else
	{
		return false;
	}
}


//Custom PID

bool Elevator::AtPosition(double setpoint)
{
	int tol=10; //number of encoder counts for tolerance
//	double setpoint_val;
	//setpoint_val=LEVEL_ZERO;
//	switch (setpoint)
//	{
//	case 0:
//		setpoint_val=LEVEL_ZERO;
//		break;
//	case 1:
//		setpoint_val=LEVEL_ONE;
//		break;
//	case 2:
//		setpoint_val=LEVEL_TWO;
//		break;
//	case 3:
//		setpoint_val=LEVEL_THREE;
//		break;
//	case 4:
//		setpoint_val=LEVEL_FOUR;
//		break;
//	case 5:
//		setpoint_val=LEVEL_FIVE;
//		break;
//	case 6:
//		setpoint_val=LEVEL_SIX;
//		break;
//	case 7:
//		setpoint_val=LEVEL_SEVEN;
//		break;
//	}

	if(abs((setpoint-elevatorEncoder.Get())) <= tol) //See if ABS function is available
	{
		//return true;
		return true;
	}
	else
	{
		return false;
	}
}

bool Elevator::BottomlimitHit() //Returns sign of the error
{
	if(leftLowerLimit.Get() == true || rightLowerLimit.Get() == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Elevator::Calibrate()
{
	if (BottomlimitHit()==true)
	{
		elevatorMotor.SetSpeed(0);
		elevatorEncoder.Reset();
		return true;
	}
	else
	{
		elevatorMotor.SetSpeed(-.3);
		return false;
	}
}

void Elevator::Reset()
{
	elevatorEncoder.Reset();
}

bool Elevator::Jamming(double elevatormotorspeed) // checks to see if the elevator is jamming into a tote as it goes down
{
	//double ElevatorSpeed;
	//bool ElevatorDirection;
	ElevatorDirection = elevatorEncoder.GetDirection(); // returns true of up and false if down
	ElevatorSpeed = elevatorEncoder.GetRate(); // Get the encoder rate

	if(ElevatorSpeed > -25) // Checks if elevator is moving and if its direction is down
	{
		SmartDashboard::PutNumber("Elevespeed", elevatorEncoder.GetRate());
		//power_to_speed_ratio = elevatormotorspeed/ElevatorSpeed; // ElevatorSpeed and elevatormotorspeed should be negative to result in ratio being positive
		power_to_speed_ratio = ElevatorSpeed;
		if(elevatormotorspeed < -.6) // Checks to see if power to speed ratio has been exceeded
		{
			return false;
		}

		else {

			return false; // return false because it is going down and not crashing
		}
	}
	else
	{
		return false; // return false because it is going up
	}

}

bool Elevator::setPID(double max_out, double min_out, int setLevel)
{
	double setpoint;
	setpoint=LevelHeight(setLevel); //Set point value in encoder counts

	if(AtPosition(setpoint) == true)
	{
		elevatorMotor.SetSpeed(0); //Disable
		ResetPID(); // Reset all initial values once PID reaches setpoint
		return true; // Return false to disable PID at setpoint.
	}

	else
	{
		elevatormotorspeed = PID(max_out, min_out, setpoint); //Run the PID
		if(Jamming(elevatormotorspeed)==true)
		{

			elevatorMotor.SetSpeed(0); // For now, just stope the motor when jamming
			ResetPID();
			return false; //Set to true if you want to target a specific unjam point
		}

		else {

			if(elevatormotorspeed > 0.0 && ToplimitHit() == true)
			{
				elevatorMotor.SetSpeed(0); //Disable
				ResetPID(); // Reset all initial values once PID reaches setpoint
				return false;
			}
			else if( setLevel == 0 && BottomlimitHit() == true)
			{
				elevatorMotor.SetSpeed(0); //Disable
				ResetPID(); // Reset all initial values once PID reaches setpoint
				return false;
			}
			else
			{
				elevatorMotor.SetSpeed(elevatormotorspeed); // Set motor speed
			}
			SmartDashboard::PutNumber("Elevator Motor Power", elevatormotorspeed);
			return true;
		}
	}
}

bool Elevator::ToplimitHit() //Returns sign of the error
{
	if(rightUpperLimit.Get() == true || leftUpperLimit.Get())
	{
		return true;
	}
	else
	{
		return false;
	}
}

double Elevator::LevelHeight(int setLevel)
{
	if (setLevel >= minLevel() && setLevel <= maxLevel()) // Makes sure level exists -- because reasons
	{
		switch (setLevel)
		{
			case 0:
				return LEVEL_ZERO;
				//destinationFloor = 0;
			break;

			case 1:
				return LEVEL_ONE;
			//	destinationFloor = 1;
			break;

			case 2:
				return LEVEL_TWO;
			//	destinationFloor = 2;
			break;

			case 3:
				return LEVEL_THREE;
			//	destinationFloor = 3;

			break;

			case 4:
				return LEVEL_FOUR;
			//	destinationFloor = 4;

			break;

			case 5:
				return LEVEL_FIVE;
			//	destinationFloor = 5;

			break;

			case 6:

				return LEVEL_SIX;
			//	destinationFloor = 6;

			break;

			case 7:

				return LEVEL_SEVEN;
			//	destinationFloor = 7; // Don't need these

			break;

		}
	}

	else if (setLevel == 8)
	{
		return LEVEL_LOAD;
	}

	else
	{
		// This condition should not be possible
	}

}

double Elevator::PID(double max_out, double min_out, double m_setpoint)
{
	double kp=6;
	double kd=3;
	//double kd = 1;
	double ki=0.0;
	double norm = 893.0;

	SmartDashboard::PutNumber("Error", p_error);
	SmartDashboard::PutNumber("setpoint", elevatorEncoder.GetDistance());
	SmartDashboard::PutNumber("PID", PID_out);

	p_error=m_setpoint-elevatorEncoder.Get();
	if (abs(p_error)<=0)
	{
		p_error =0;
	}
	p_error=(p_error/norm); //normalize the error (0-100 from bottom to top)
	d_error=p_error-error_prev;
	i_error=i_error+p_error;

	PID_out= kp*p_error+kd*d_error+ki*i_error;

	if(PID_out > max_out)
	{
		PID_out= max_out;
	}
	else if (PID_out< min_out)
	{
		PID_out = min_out;
	}
	error_prev=p_error;
	return PID_out;
}

int Elevator::maxLevel()
{
	return MAX_LEVEL;
}

int Elevator::minLevel()
{
	return MIN_LEVEL;
}

void Elevator::ResetPID()
{
	p_error=0.0;
	error_prev=0.0;
	i_error = 0.0;
	PID_out=0.0;
}

//Unused Functions


void Elevator::Execute()
{
	Timer elevatorRest;
	SmartDashboard::PutNumber("Current Encoder Pos", elevatorEncoder.Get());
	SmartDashboard::PutBoolean("BLS", BottomlimitHit());
	SmartDashboard::PutBoolean("TLS", ToplimitHit());
	SmartDashboard::PutNumber("Jam Ratio", power_to_speed_ratio);
	SmartDashboard::PutNumber("Elevespeed", elevatorEncoder.GetRate());
	//SmartDashboard::PutNumber("Current Encoder Position", elevatorEncoder.GetDistance());
//#ifndef LIMIT_SWITCHES_ARE_MISSING
//	if (leftLowerLimit.Get() == true || rightLowerLimit.Get() == true)
//	{
//		elevatorEncoder.Reset();
//		elevatorPid.Reset();
//	}
//#endif
//	elevatorRest.Start();
//	if (IsAtLevel() == true)
//	{
//		elevatorPid.Disable();
//		if (elevatorRest.Get() > 0.5)
//		{
//		BrakeOn();
//		}
//	}

	/*if (AtSetPoint() == true)
	{
		customPID.Disable();
	}*/
	//This will have to keep track while the elevator is reseting watching for the limit switch, once it hits the switch then reset the encoder and pid, then set the pid setpoint to 0 and enable it
}
//
//void Elevator::stopPID()
//{
//elevatorMotor.SetSpeed(0);
//}
//
//void Elevator::Reset()
//{
//
//
////Send the motor down slow-ishly use execute function to check for hitting the lower limit switch
////This would need to be called before your first go to command, so you should keep track of whether or not it has been initialized with a boolean, and in the set routine call reset if it hasn't been done
//}
//
//void Elevator::TwitchFromDown()	//When we stop LOWERING the elevator, it raises the elevator slightly before putting the brake on
//{
//	static float encoderObjectiveCount_TfD;	//Does it need to be static?
//	if (isStopChecked == false)
//	{
//		encoderObjectiveCount_TfD = elevatorEncoder.Get();	//Is this right, or do we need to add some number of counts?
//		isStopChecked = true;
//	}
//
//	if (isStopChecked == true && (elevatorEncoder.Get() < encoderObjectiveCount_TfD))
//	{
//		elevatorMotor.SetSpeed(0.4);	//TODO: probably need to calibrate this speed
//		BrakeOn();
//	}
//	else if (elevatorEncoder.Get() >= encoderObjectiveCount_TfD)
//	{
//		elevatorMotor.SetSpeed(0);
//		isStopChecked = false;	//Is this the right place to put this line? -Ben
//		return;					//Is this necessary?
//	}
//}
//
//void Elevator::DisablePid()
//{
//	elevatorPid.Disable();
//}
//
//
//void Elevator::SetLevel(int destinationLevel)
//{
//	if (destinationLevel >= 0 && destinationLevel <= 7) // Makes sure level exists -- because reasons
//	{
//		elevatorPid.Enable();
//		switch (destinationLevel)
//		{
//		case 0:
//			if (leftLowerLimit.Get() == false || rightLowerLimit.Get() == false)
//			{
//				elevatorPid.SetSetpoint(LEVEL_ZERO);
//				destinationFloor = 0;
//			}
//			else
//			{
//				elevatorMotor.SetSpeed(0);	//Do we need this? It would be useful, wouldn't it?
////				elevatorEncoder.Reset();
//			}
//			break;
//
//		case 1:
//			elevatorPid.SetSetpoint(LEVEL_ONE); //This is a dummy value right now. We will need to determine the values for these constants
//			destinationFloor = 1;
//			break;
//		case 2:
//			elevatorPid.SetSetpoint(LEVEL_TWO); //This is a dummy value right now. We will need to determine the values for these constants
//			destinationFloor = 2;
//			break;
//		case 3:
//			elevatorPid.SetSetpoint(LEVEL_THREE); //This is a dummy value right now. We will need to determine the values for these constants
//			destinationFloor = 3;
//			break;
//		case 4:
//			elevatorPid.SetSetpoint(LEVEL_FOUR); //This is a dummy value right now. We will need to determine the values for these constants
//			destinationFloor = 4;
//			break;
//		case 5:
//			elevatorPid.SetSetpoint(LEVEL_FIVE); //This is a dummy value right now. We will need to determine the values for these constants
//			destinationFloor = 5;
//			break;
//		case 6:
//			if (leftUpperLimit.Get() == false && rightUpperLimit.Get() == false)
//			{
//				elevatorPid.SetSetpoint(LEVEL_SIX);
//				destinationFloor = 6;
//			}
//			else
//			{
//				elevatorMotor.SetSpeed(0);	//Do we need this? It would be useful, wouldn't it?  NO!!!
//			}
//			break;
//
//		}
//	}
//	else
//	{
//		elevatorMotor.SetSpeed(0);	//Do we need this? It would be useful, wouldn't it?  NO!!!
//	}
//
//}
