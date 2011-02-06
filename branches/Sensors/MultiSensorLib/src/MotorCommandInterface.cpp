/*
 * MotorCommandInterface.cpp
 *
 *  Created on: Jan 12, 2011
 *      Author: tallred3
 */

#include "MotorCommandInterface.h"
#include "WiiController.h"

#include <iostream>
using namespace std;

MotorCommandInterface::MotorCommandInterface() { }

MotorCommandInterface::~MotorCommandInterface() { }

bool MotorCommandInterface::setVelocity( double linearVelocity, double angularVelocity ) {
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	return true;
}

bool MotorCommandInterface::getSuccess() {
	return this->success;
}

DualMotorCommand::DualMotorCommand() { }

DualMotorCommand::~DualMotorCommand() { }

void DualMotorCommand::doProcess() {
	WiiController * controller = WiiController::getReference();
	uint16_t lxaxis, lyaxis;
	uint16_t rxaxis, ryaxis;
	controller->getLeftStick( lxaxis, lyaxis );
	controller->getRightStick( rxaxis, ryaxis );

	double lspeed = ((lyaxis-32)/32.0)*1000; //need to divide by 32 because the l stick gives us 6 bits precision
	double rspeed = ((ryaxis-16)/16.0)*1000; //need to divide by 16 because the r stick gives us 5 bits precision

	cout << "lyaxis:" << lyaxis << "Lspeed " << (int)lspeed << endl;
	cout << "ryaxis:" << ryaxis << "Rspeed " << (int)rspeed << endl;

//		MotorController::instance()->setSpeed(MotorController::Channel1, (int)lspeed);
//		MotorController::instance()->setSpeed(MotorController::Channel2, (int)rspeed);
//		MotorController::instance()->doProcess();

}

DummyMotorCommand::DummyMotorCommand() {

}

DummyMotorCommand::~DummyMotorCommand() {

}

void DummyMotorCommand::doProcess() {

}
