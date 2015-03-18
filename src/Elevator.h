/*
 * Elevator.h
 *
 *  Created on: Feb 7, 2015
 *      Author: Bill
 */
#include "Wpilib.h"

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

// 1 inch = 29.15 pulses

#define LEVEL_ZERO .25*29.15 //
#define LEVEL_ONE 2.5*29.15 //
#define LEVEL_TWO 7*29.15//
#define LEVEL_THREE 10.5*29.15//
#define LEVEL_FOUR 16*29.15//
#define LEVEL_FIVE 19*29.15//
#define LEVEL_SIX 24.5*29.15//
#define LEVEL_SEVEN 31.25*29.15//
//#define LEVEL_LOAD  3.5*29.15
#define LEVEL_LOAD  4.8*29.15
#define MAX_LEVEL 7//
#define MIN_LEVEL 0//
 // Defines the maximum value of the (motor power[0-1])/(Elevator Speed[counts/sec]) ratio

class Elevator : public IterativeRobot
{

private:
	Solenoid elevatorExtend;
	Solenoid elevatorRetract;
	Solenoid elevatorBrakeExtend;
	Solenoid elevatorBrakeRetract;
	Victor elevatorMotor;
	Encoder elevatorEncoder;
	DigitalInput leftLowerLimit;
	DigitalInput rightLowerLimit;
	DigitalInput leftUpperLimit;
	DigitalInput rightUpperLimit;

	// = new DigitalInput(0)
	int destinationLevel;
	int destinationFloor;
	bool isCountSet;
	bool isStopChecked;
	//float destinationPulse;
	PIDController elevatorPid;

public: //Elevator Extend

	Elevator(int elevator_extend, int elevator_retract, int elevator_brake_extend, int elevator_brake_retract, int elevator_motor, int elevator_encoder_A, int elevator_encoder_B, int lower_left_limit, int lower_right_limit, int upper_left_limit, int upper_right_limit);

	bool IsCrashing();

	void BrakeOff();
	void BrakeOn();
	void ExtendElevator();
	void RetractElevator();

//Unused functions
	void Execute(); // Questionable. Probably remove
//	void Reset();
//	void TwitchFromDown();
//	void SetLevel(int destinationLevel);
//	void TestElevatorMotor(float motorSpeed);
//	void DisablePid();
//	bool IsAtLevel();
//	void SetPID(float p, float i, float d){elevatorPid.SetPID(p,i,d);}
//	void stopPID();

//Custom PID

	//Variables
	double p_error=0.0;
	double d_error=0.0;
	double i_error=0.0;
	double error_prev=0.0;
	double PID_out=0;
	double elevatormotorspeed;
	double power_to_speed_ratio;
	bool ElevatorDirection;
	double ElevatorSpeed;
	double MAX_RATIO = .15;
//	double ElevatorSpeed;

	//Objects
	bool AtPosition(double setpoint);
	bool BottomlimitHit();
	bool Calibrate();
	void Reset();
	bool Jamming(double elevatormotorspeed);
	bool setPID(double max_out, double min_out, int setLevel);
	bool ToplimitHit();
	double LevelHeight(int setLevel);
	double PID(double max_out, double min_out, double setpoint);
	int maxLevel();
	int minLevel();
	void ResetPID();

};

#endif /* SRC_ELEVATOR_H_ */
