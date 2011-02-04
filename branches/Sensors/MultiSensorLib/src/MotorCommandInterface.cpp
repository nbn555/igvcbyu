/*
 * MotorCommandInterface.cpp
 *
 *  Created on: Jan 12, 2011
 *      Author: tallred3
 */

#include "MotorCommandInterface.h"

MotorCommandInterface::MotorCommandInterface() {

}

MotorCommandInterface::~MotorCommandInterface() {
}

bool MotorCommandInterface::setVelocity( double linearVelocity, double angularVelocity ) {
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	return true;
}

bool MotorCommandInterface::getSuccess() {
	return this->success;
}

DummyMotorCommand::DummyMotorCommand() {

}

DummyMotorCommand::~DummyMotorCommand() {

}

void DummyMotorCommand::doProcess() {

}
