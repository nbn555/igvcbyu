/*
 * MotorCommandInterface.cpp
 *
 *  Created on: Jan 12, 2011
 *      Author: tallred3
 */

#include "MotorCommandInterface.h"
#include "logging.h"

#include <iostream>
using namespace std;

MotorCommandInterface::MotorCommandInterface(){ }

MotorCommandInterface::~MotorCommandInterface() { }

bool MotorCommandInterface::setVelocity( double linearVelocity, double angularVelocity ) {

	const double zeroThreshold = .001;

	this->linearVelocity = (abs(linearVelocity) < zeroThreshold) && (abs(angularVelocity) > zeroThreshold) ? -.2 : linearVelocity;
	this->angularVelocity = angularVelocity;
	return true;
}

bool MotorCommandInterface::getSuccess() {
	return this->success;
}

MotorCommand::MotorCommand( double lC, double aC): linearConst(lC), angConst(aC) {
	MotorController::instance()->setMixingMode(1);  //Set the mixing mode to 1 so it is in mixing mode
	MotorController::instance()->setOperatingMode(1); //Set the operating mode so the robot is in closed loop speed
	MotorController::instance()->saveConfig();
}

void MotorCommand::doProcess() {
	int rmp1= linearConst * this->linearVelocity;
	int rmp2= angConst * this->angularVelocity;
	if(!(MotorController::instance()->setSpeed(MotorController::Channel1, rmp1)&&MotorController::instance()->setSpeed(MotorController::Channel2, rmp2)))
		this->success = false;
	MotorController::instance()->doProcess();
	this->success =  true;

}

MotorCommand::~MotorCommand() {
}

DualMotorCommand::DualMotorCommand() {
	MotorController::instance()->setMixingMode(1);  //Set the mixing mode to 0 so the controller is in separate mode or tank Mode
	MotorController::instance()->setOperatingMode(0); //Set the operating mode so the robot is in open loop in dual motor command
	this->linearVelocity = 16;
	this->angularVelocity = 32;
	MotorController::instance()->saveConfig();
}

DualMotorCommand::~DualMotorCommand() { }

void DualMotorCommand::doProcess() {
	uint16_t lyaxis = this->linearVelocity;	//The wii controller uses separate mode so linear and angular velocities should really
											//be thought of as left and right respectively
	uint16_t ryaxis = this->angularVelocity;

	double lspeed = ((lyaxis-32)/32.0)*1000; //need to divide by 32 because the l stick gives us 6 bits precision
	double rspeed = ((ryaxis-16)/16.0)*1000; //need to divide by 16 because the r stick gives us 5 bits precision

	LOG_MOTOR(DEBUG3) << "lyaxis:" << lyaxis << " Lspeed " << (int)lspeed << endl;
	LOG_MOTOR(DEBUG3) << "ryaxis:" << ryaxis << " Rspeed " << (int)rspeed << endl;

	MotorController::instance()->setSpeed(MotorController::Channel1, (int)lspeed);
	MotorController::instance()->setSpeed(MotorController::Channel2, (int)rspeed);
	MotorController::instance()->doProcess();

}

DummyMotorCommand::DummyMotorCommand() {
	MotorController::instance()->setMixingMode(1);  //Set the mixing mode to 0 so the controller is in separate mode or tank Mode
	MotorController::instance()->setOperatingMode(0); //Set the operating mode so the robot is in open loop in dual motor command
	MotorController::instance()->saveConfig();
}

DummyMotorCommand::~DummyMotorCommand() {

}

void DummyMotorCommand::doProcess() {

}
