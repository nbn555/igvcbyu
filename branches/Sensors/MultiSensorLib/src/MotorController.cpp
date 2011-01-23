/**
 * @file MotorController.cpp
 * @brief This file contains implementation code for interfacing
 * to the roboteq 2440 motor controller
 *
 * @date Dec 11, 2010
 * @author T. Eldon Allred
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

void MotorController::doProcess() {
		stringstream parser;
		parser << "!M " << this->motor1Speed << " " << this->motor2Speed << "\n\r";
		this->sendCommand(parser.str());
		this->motor1Speed = 0;
		this->motor2Speed = 0;
}

bool MotorController::SetAcceleration(MotorChannel channel, double acceleration) {
	assert(false);//TODO implelment Set Acceleration
	return false;
}

bool MotorController::SetDeceleration(MotorChannel channel, double deceration) {
	assert(false);//TODO implement Set Deceleration
	return false;
}

bool MotorController::emergencyStop() {
	return this->sendCommand("!EX\n\r");
}

bool MotorController::clearEmergencyStop() {
	return this->sendCommand("!MG\n\r");
}

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


		//Need to add rval in case motor1 speed isn't in a valid range but motor 2 is
		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) && rval ) {
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
	assert(false);//TODO implement set Encoder Counter
	return false;
}

bool MotorController::getMotorAmps( int & motor1Amps, int & motor2Amps ) {
	assert(false);//TODO implement get Motor Amps
	return false;
}

bool MotorController::getAnalogInputs( int input, int & value ) {
	assert(false);//TODO implement getAnalogInputs
	return false;
}

bool MotorController::getBatteryAmps( int & motor1Amps, int & motor2Amps ) {
	assert(false);//TODO implement get Battery Amps
	return false;
}

bool MotorController::getAbsoluteEncoderCount( int & ch1, int & ch2 ) {
	assert(false);//TODO implement getAbsoluteEncoderCount
	return false;
}

bool MotorController::getRelativeEncoderCount( int & ch1, int & ch2 ) {
	this->sendCommand("?CR\n\r");//TODO getRelativeEncoderCount
	assert(false);
	return false;
}

bool MotorController::getEncoderSpeed( MotorChannel channel, int & speed ) {
	assert(false);//TODO implement getEncoderSpeed
	return false;
}

bool MotorController::getTemperature( int & ch1, int & ch2 ) {
	this->sendCommand("?T\n\r");//TODO
	assert(false);
	return false;
}

bool MotorController::getTime( int & hours, int & minutes, int & seconds ) {
	assert(false);//TODO implement getTime
	return false;
}

bool MotorController::getVoltages( int & driverVolt, int & batteryVolt, int & v5out ) {
	assert(false);//TODO implement getVoltages
	return false;
}

bool MotorController::loadEEPROMSettings() {
	assert(false);//TODO implement loadEEPROMSettings
	return false;
}

bool MotorController::failSafeReset() {
	assert(false);//TODO implement failSafeReset
	return false;
}

bool MotorController::saveConfigSettings() {
	assert(false);//TODO implement saveConfigSettings
	return false;
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

bool MotorController::setCommandLinearity(int linearity ) {
	assert(false);//TODO setCommandLinearity
	return false;
}

bool MotorController::setCommandPriority( int signal, int priority ) {
	assert(false);//TODO setCommandPriority
	return false;
}

bool MotorController::setSerialWatchDogTimer( int time = 1000 ) {
	assert(false);//TODO setSerialWatchDogTimer
	return false;
}

bool MotorController::setEncoderPPR( int ppr ) {
	assert(false);//TODO setEncoderPPR
	return false;
}

bool MotorController::setOverVoltageLimit( double limit ) {
	assert(false);//TODO setOverVoltageLimit
	return false;
}

bool MotorController::setUnderVoltageLimit( double limit ) {
	assert(false);//TODO setUnderVoltageLimit
	return false;
}

bool MotorController::setShortCircuitDetectionThreshold( int value ) {
	assert(false);//TODO setShortCircuitDetectionThreshold
	return false;
}

bool MotorController::getMotorLimits( int & m1Max, int & m2Max, int & m1Min, int & m2Min ) {
	m1Max = this->motor1SpeedMax;
	m2Max = this->motor2SpeedMax;
	m1Min = this->motor1SpeedMin;
	m2Min = this->motor2SpeedMin;
	return true;
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

bool MotorController::getFaultFlags() {
	assert(false);//TODO getFaultFlags
}

bool MotorController::getStatusFlags() {
	assert(false);//TODO getStatusFlags
	return false;
}

bool MotorController::getControlUnitType() {
	assert(false);//TODO getControlUnitType
	return false;
}

bool MotorController::clearBufferHistory() {
	assert(false);//TODO clearBufferHistory
	return false;
}

