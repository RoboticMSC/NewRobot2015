/*
 * Rollers.h
 *
 *  Created on: Feb 7, 2015
 *      Author: Bill
 */

#ifndef SRC_ROLLERS_H_
#define SRC_ROLLERS_H_
#include "Wpilib.h"

class Rollers : public IterativeRobot
{
private:
	Solenoid rollersOpen;
	Solenoid rollersClose;
	VictorSP rollerPolarRight;
	VictorSP rollerPolarLeft;

public:
Rollers(int rollers_extend, int rollers_retract, int roller_right_motor, int roller_left_motor);
void OpenRollers();
void CloseRollers();
void Eat(float rollerSpeed); //Aka, intake. Kids these days
void Barf(float rollerSpeed); // Aka, eject. ^^
void PushRight(float rollerSpeed);
void PushLeft(float rollerSpeed);
void RollersIdle(); // Turn off motors

};

#endif /* SRC_ROLLERS_H_ */
