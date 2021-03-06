/*
 * Rollers.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: Bill
 */
#include "Rollers.h"

Rollers::Rollers(int roller_extend, int roller_retract, int roller_right_motor, int roller_left_motor):
rollersOpen(0, roller_extend),
rollersClose(0, roller_retract),
rollerPolarRight(roller_right_motor),
rollerPolarLeft(roller_left_motor)
{

}

void Rollers::OpenRollers()
{
	rollersOpen.Set(true);
	rollersClose.Set(false);
}

void Rollers::CloseRollers()
{
	rollersOpen.Set(false);
	rollersClose.Set(true);
}

void Rollers::Eat(float rollerSpeed)
{
	rollerPolarRight.SetSpeed(rollerSpeed);
	rollerPolarLeft.SetSpeed(rollerSpeed);
}

void Rollers::Barf(float rollerSpeed)
{
	rollerPolarRight.SetSpeed(-rollerSpeed);
	rollerPolarLeft.SetSpeed(-rollerSpeed);
}

void Rollers::PushRight(float rollerSpeed)
{
	rollerPolarRight.SetSpeed(-rollerSpeed);
	rollerPolarLeft.SetSpeed(rollerSpeed);
}

void Rollers::PushLeft(float rollerSpeed)
{
	rollerPolarRight.SetSpeed(-rollerSpeed);
	rollerPolarLeft.SetSpeed(rollerSpeed);
}

void Rollers::RollersIdle()
{
	rollerPolarRight.SetSpeed(0);
	rollerPolarLeft.SetSpeed(0);
}
