/*
 * MotorController.cpp
 *
 *  Created on: Dec 11, 2010
 *      Author: T. Eldon Allred
 */

#include <iostream>
#include <sstream>
#include <string>
#include <cassert>

#include <MotorController.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;

MotorController::MotorController( string portName, bool enableEcho, int motor1Max, int motor2Max, int motor1Min, int motor2Min ):
		echoEnabled(enableEcho),
		motor1Speed(0), motor2Speed(0),
		motor1SpeedMax(motor1Max), motor2SpeedMax(motor2Max),
		motor1SpeedMin(motor1Min), motor2SpeedMin(motor2Min)
{
	this->serialPort.open(portName);
	this->serialPort.setConfig(115200, 0, 8, 1, false);//Non configurable port setting see the roboteq manual

	if(echoEnabled) {
		this->enableSerialEcho(); //enable the serial to echo the commands back used for error checking
	}else {
		this->disableSerialEcho();//disable serial port echo
	}

	this->sendCommand("!H\n\r");//Load preset values for all counters
	this->sendCommand("!C 1 0\n\r");//clear out the first encoder counter
	this->sendCommand("!C 2 0\n\r");//clear out the second encoder counter

};

bool MotorController::setSpeed( MotorChannel channel, int value ) {
	bool rval = false;
	switch(channel) {
	case Channel1:
		if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}

		break;
	case Channel2:
		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) ) {
			this->motor2Speed = value;
			rval = true;
		}
		break;
	case BothChannels:
		if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}


		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) ) {
			this->motor2Speed = value;
			rval = true;
		}
		break;
	default:
		cerr << "Invalid Motor Channel" << endl;
		break;
	}

	return rval;
}

bool MotorController::setEncoderCounter( MotorChannel channel, int value ) {
	assert(false);//TODO
	return false;
}

bool MotorController::getRelativeEncoderCount( int & ch1, int & ch2 ) {
	this->sendCommand("?CR\n\r");//TODO
	assert(false);
	return false;
}

bool MotorController::getTemperature( int & ch1, int & ch2 ) {
	this->sendCommand("?T\n\r");//TODO
	assert(false);
	return false;
}

bool MotorController::emergencyStop() {
	return this->sendCommand("!EX\n\r");
}

bool MotorController::clearEmergencyStop() {
	return this->sendCommand("!MG\n\r");
}

bool MotorController::reset() {
	stringstream stream;
	stream << "%RESET " << MotorController::MOTOR_SECRET_KEY << "\n\r";
	return this->sendCommand(stream.str());
}

bool MotorController::setTime( int hours, int minutes, int seconds) {
	stringstream stream;
	stream << "%STIME " << hours << ":" << minutes << ":" << seconds << "\n\r";
	return this->sendCommand(stream.str());
}

bool MotorController::getMotorLimits( int & m1Max, int & m2Max, int & m1Min, int & m2Min ) {
	m1Max = this->motor1SpeedMax;
	m2Max = this->motor2SpeedMax;
	m1Min = this->motor1SpeedMin;
	m2Min = this->motor2SpeedMin;
	return true;
}

void MotorController::doProcess() {
		stringstream parser;
		parser << "!M " << this->motor1Speed << " " << this->motor2Speed << "\n\r";
		this->sendCommand(parser.str());
}

bool MotorController::assertValidMotorRange( int max, int min, int value ) const {
	return ( value > min && value < max );
}

bool MotorController::assertValidVoltage() {
	this->sendCommand("?V\n\r");//TODO parse the voltage command and check it is safe
	assert(false);
	return false;
}

bool MotorController::sendCommand( std::string command ) {
	bool rval = false;
	if( this->serialPort.isOpen() ) {
		cout << "Command" << command.c_str() << endl;
		this->serialPort.Write(command.c_str(), command.length());
	}
	if( this->echoEnabled ) {
		//check if the response is valid and return
	}
	rval = true;//TODO change this to detect verification of the command
	return rval;
}

void MotorController::enableSerialEcho() {
	this->sendCommand("^ECHOF 0\n\r");
}

void MotorController::disableSerialEcho() {
	this->sendCommand("^ECHOF 1\n\r");
}
