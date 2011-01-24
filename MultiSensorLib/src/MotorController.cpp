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

bool MotorController::setAcceleration(MotorChannel channel, double acceleration) {
	assert(false);//TODO implelment Set Acceleration
	return false;
}

bool MotorController::setDeceleration(MotorChannel channel, double deceration) {
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


		//Needed to add rval in case motor1 speed isn't in a valid range but motor 2 is
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
	bool rval = false;
	stringstream parser;

	switch(channel) {
	case Channel1:
		parser << "!C 1 " << value << '\r';
		rval = true;
		break;
	case Channel2:
		parser << "!C 2 " << value << '\r';
		rval = true;
		break;
	case BothChannels:
		parser << "!C 1 " << value << '\r' << "!C 2 " << value << '\r';
		rval = true;
		break;
	default:
		cerr << "Invalid motor channel for encoder set value" << endl;
	}

	this->serialPort.WriteBuffer(parser.str().c_str(),parser.str().length());

	return rval;
}

bool MotorController::getMotorAmps( double & motor1Amps, double & motor2Amps ) {
	string command = "?A\r";
	this->serialPort.Write(command.c_str(),command.length());

	int channel1;
	int channel2;

	this->responseParserDual(channel1, channel2);

	motor1Amps = channel1/10; //The value returned is in deciamps
	motor2Amps = channel2/10;

	return true;
}

bool MotorController::getAnalogInputs( int input, int & value ) {
	assert(false);//TODO implement getAnalogInputs
	return false;
}

bool MotorController::getBatteryAmps( double & motor1Amps, double & motor2Amps ) {
	int channel1;
	int channel2;

	this->sendCommand("?BA\r");
	bool rval = this->responseParserDual(channel1, channel2);

	motor1Amps = channel1/10; //The value returned is in deciamps converting to amps
	motor2Amps = channel2/10;

	return rval;
}

bool MotorController::getAbsoluteEncoderCount( int & ch1, int & ch2 ) {
	this->sendCommand("?C\r");
	return this->responseParserDual(ch1, ch2);
}

bool MotorController::getRelativeEncoderCount( int & ch1, int & ch2 ) {
	this->sendCommand("?CR\n\r");
	return this->responseParserDual(ch1,ch2);
}

bool MotorController::getEncoderSpeed( MotorChannel channel, int & speed ) {
	assert(false);//TODO implement getEncoderSpeed
	return false;
}

bool MotorController::getTemperature( int & ch1, int & ch2, int & ic ) {
	this->sendCommand("?T 1\r");				//Could be optimized to only send one string
	return this->responseParserTrio(ic,ch1,ch2);
}

bool MotorController::getTime( int & hours, int & minutes, int & seconds ) {
	this->sendCommand("?TM\r");
	return this->responseParserTrio(hours,minutes,seconds);
}

bool MotorController::getVoltages( double & driverVolt, double & batteryVolt, double & v5out ) {
	int dV;
	int bV;
	int v5;

	this->sendCommand("?V\r");
	bool rval = this->responseParserTrio(dV,bV,v5);
	driverVolt = dV/10; //returned from motor controller in decivolts
	batteryVolt = bV/10; //returned from motor controller in decivolts
	v5out = v5/1000; //returned from motor controller in millivolts
	return rval;
}

bool MotorController::loadEEPROMSettings() {
	return this->sendCommand("%EELD\r");
}

bool MotorController::failSafeReset() {
	//return this->sendCommand("%EERST 321654987\r");
	return false;
}

bool MotorController::saveConfigSettings() {
	assert(false);//TODO implement saveConfigSettings
	return false;
}

bool MotorController::reset() {
	stringstream stream;
	stream << "%RESET " << MotorController::MOTOR_SECRET_KEY << "\r";
	return this->sendCommand(stream.str());
}

bool MotorController::setTime( int hours, int minutes, int seconds) {
	stringstream stream;
	stream << "%STIME " << hours << ":" << minutes << ":" << seconds << "\r";
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

bool MotorController::setSerialWatchDogTimer( int time) {
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
	return false;
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

bool MotorController::responseParserDual( int & t1, int & t2 ) {
	stringstream responseParser( this->serialPort.ReadString() );

	while( '=' != responseParser.get() );

	responseParser >> t1;
	responseParser.get(); //Parse off the :
	responseParser >> t2;

	return true;

}

bool MotorController::responseParserTrio( int & t1, int & t2, int & t3 ) {

	stringstream responseParser( this->serialPort.ReadString() );

	while( '=' != responseParser.get() );

	responseParser >> t1;
	responseParser.get(); //Parse off the :
	responseParser >> t2;
	responseParser.get(); //Parse off the :
	responseParser >> t3;

	return true;

}
