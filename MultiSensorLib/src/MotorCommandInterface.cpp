/*
 * MotorCommandInterface.cpp
 *
 *  Created on: Jan 12, 2011
 *      Author: tallred3
 */

#include "MotorCommandInterface.h"

MotorCommandInterface::MotorCommandInterface( MotorController * mP ): motorPtr(mP) {

}

MotorCommandInterface::~MotorCommandInterface() {
}

bool MotorCommandInterface::setVelocity( double linearVelocity, double angularVelocity ) {
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	return true;
}
