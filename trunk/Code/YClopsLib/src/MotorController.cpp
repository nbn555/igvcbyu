/**t
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
#include "logging.h"

#include "MotorController.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

MotorController * MotorController::mc = NULL;
CConfigFileBase * MotorController::config = NULL;

MotorController * MotorController::instance() {
	if( NULL == MotorController::mc ) {
		LOG_MOTOR(DEBUG4) << "Initializing Motor Controller" << endl;
		MotorController::mc = new MotorController();
	}
	return MotorController::mc;
}

void MotorController::setConfigFile( mrpt::utils::CConfigFileBase * configFile ) {
	MotorController::config = configFile;
}

MotorController::MotorController( mrpt::utils::CConfigFileBase * conf ):
		echoEnabled(conf->read_bool("MOTOR", "ECHO_ENABLED", true)),
		motor1Speed(0), motor2Speed(0),
		motor1SpeedMax(conf->read_int("MOTOR", "MAX_FORWARD_LEFT", 0) ),  //If the config file isn't correct we don't want to be able to do anything for safety reasons
		motor2SpeedMax(conf->read_int("MOTOR", "MAX_FORWARD_RIGHT", 0 ) ),
		motor1SpeedMin(conf->read_int("MOTOR", "MIN_REVERSE_LEFT", 0 ) ),
		motor2SpeedMin(conf->read_int("MOTOR", "MIN_REVERSE_RIGHT", 0) )
{

	string portName = conf->read_string("MOTOR", "COM_port_LIN", "/dev/ttyS1" );

	LOG_MOTOR(DEBUG4) << "Connecting to Motor Controller on port: " << portName << endl;

	this->serialPort.open(portName);

	this->serialPort.setConfig(115200, 0, 8, 1, false);//Non configurable port setting see the roboteq manual

//	if( !this->reset() ) {
//		LOG_MOTOR(FATAL) << "Motor Controller Failed to successfully reset" << endl;
//	}

	if(echoEnabled) {
		this->enableSerialEcho(); //enable the serial to echo the commands back used for error checking
	}else {
		this->disableSerialEcho();//disable serial port echo
	}

	if( motor1SpeedMax <= motor1SpeedMin ) {
		LOG_MOTOR(FATAL) << "Motor Controller channel 1 malconfigured motor1SpeedMax: " << motor1SpeedMax
				<< "motor1SpeedMin: " << motor1SpeedMin << endl;
		this->emergencyStop();
	}

	if( motor2SpeedMax <= motor2SpeedMin ) {
		LOG_MOTOR(FATAL) << "Motor Controller channel 2 malconfigured motor2SpeedMax: " << motor2SpeedMax
						<< "motor2SpeedMin: " << motor2SpeedMin << endl;
		this->emergencyStop();
	}

//	string loopMode = conf->read_string("MOTOR", "LOOP_MODE", "OPEN_LOOP");

//	LOG_MOTOR(DEBUG4) << "Setting motor channels to " << loopMode << endl;

//	this->setLoopMode(  BothChannels, loopMode );

	LOG_MOTOR(DEBUG4) << "Exiting Motor Controller Constructor" << endl;

};

void MotorController::doProcess() {
		stringstream parser;
		parser << "!M " << this->motor1Speed << " " << this->motor2Speed << "\n\r";
		this->sendCommand(parser.str(), "+");
		this->motor1Speed = 0;
		this->motor2Speed = 0;
}

bool MotorController::setAcceleration(MotorChannel channel, double acceleration) {
	LOG_MOTOR(FATAL) << "MotorController: Set Acceleration" << endl;//TODO Set Acceleration
	return false;
}

bool MotorController::setDeceleration(MotorChannel channel, double deceration) {
	LOG_MOTOR(FATAL) << "MotorController: Set Deceleration" << endl;//TODO Set Deceleration
	return false;
}

bool MotorController::emergencyStop() {
	LOG_MOTOR(DEBUG4) << "Setting emergency stop" << endl;
	return this->sendCommand("!EX\n\r", "+");
}

bool MotorController::clearEmergencyStop() {
	LOG_MOTOR(DEBUG4) << "Clearing emergency stop" << endl;
	return this->sendCommand("!MG\n\r", "+");
}

bool MotorController::setSpeed( MotorChannel channel, int value ) {
	bool rval = false;
	switch(channel) {
	case Channel1:
		/*if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}*/
		value = value > motor1SpeedMax ? motor1SpeedMax : value;
		value = value < motor1SpeedMin ? motor1SpeedMin : value;
		this->motor1Speed = value;
		rval = true;
		break;
	case Channel2:
		/*if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) ) {
			this->motor2Speed = value;
			rval = true;
		}*/
		value = value > motor2SpeedMax ? motor2SpeedMax : value;
		value = value < motor2SpeedMin ? motor2SpeedMin : value;
		this->motor2Speed = value;
		rval = true;
		break;
	case BothChannels:
		/*if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}


		//Needed to add rval in case motor1 speed isn't in a valid range but motor 2 is
		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) && rval ) {
			this->motor2Speed = value;
			rval = true;
		}*/
		value = value > motor1SpeedMax ? motor1SpeedMax : value;
		value = value < motor1SpeedMin ? motor1SpeedMin : value;
		this->motor1Speed = value;
		//rval = true;
		value = value > motor2SpeedMax ? motor2SpeedMax : value;
		value = value < motor2SpeedMin ? motor2SpeedMin : value;
		this->motor2Speed = value;
		rval = true;
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
		rval = this->sendCommand(parser.str(),"+");
		break;
	case Channel2:
		parser << "!C 2 " << value << '\r';
		rval = this->sendCommand(parser.str(),"+");
		break;
	case BothChannels:
		parser << "!C 1 " << value << '\r';
		rval = this->sendCommand(parser.str(),"+");
		parser << "!C 2 " << value << '\r';
		rval = rval && this->sendCommand(parser.str(),"+");
		break;
	default:
		LOG_MOTOR(ERROR) << "Invalid motor channel for encoder set value" << endl;
	}

	return rval;
}

bool MotorController::getMotorAmps( double & motor1Amps, double & motor2Amps ) {
	string command = "?A\r";
	this->serialPort.Write(command.c_str(),command.length());

	int channel1;
	int channel2;

	this->responseParser<int>( 2, &channel1, &channel2);

	motor1Amps = channel1/10; //The value returned is in deciamps
	motor2Amps = channel2/10;

	return true;
}

bool MotorController::getAnalogInputs( int input, int & value ) {
	LOG_MOTOR(FATAL) << "MotorController: getAnalogInputs" << endl;//TODO getAnalogInputs
	return false;
}

bool MotorController::getBatteryAmps( double & motor1Amps, double & motor2Amps ) {
	int channel1;
	int channel2;

	this->sendCommand("?BA\r", "");
	bool rval = this->responseParser<int>(2, &channel1, &channel2);

	motor1Amps = channel1/10.; //The value returned is in deciamps converting to amps
	motor2Amps = channel2/10.;

	return rval;
}

bool MotorController::getAbsoluteEncoderCount(int & ch1, int & ch2 ) {
	LOG_MOTOR(DEBUG4) << "GetAbsoluteEncoderCount" << endl;
	this->sendCommand("?C\r", "");
	return this->responseParser<int>(2, &ch1, &ch2);
}

bool MotorController::getRelativeEncoderCount( int & ch1, int & ch2 ) {
	this->sendCommand("?CR\r", "");
	return this->responseParser<int>(2, &ch1, &ch2);
}

bool MotorController::getEncoderSpeed( MotorChannel channel, int & speed ) {
	LOG_MOTOR(FATAL) << "MotorController: getEncoderSpeed" << endl;//TODO getEncoderSpeed
	return false;
}

bool MotorController::getTemperature( int & ch1, int & ch2, int & ic ) {
	this->sendCommand("?T 1\r", "");				//Could be optimized to only send one string
	return this->responseParser<int>(3,&ic,&ch1,&ch2);
}

bool MotorController::getTime( int & hours, int & minutes, int & seconds ) {
	this->sendCommand("?TM\r", "");
	return this->responseParser<int>(3,&hours,&minutes,&seconds);
}

bool MotorController::getVoltages( double & driverVolt, double & batteryVolt, double & v5out ) {
	int dV;
	int bV;
	int v5;

	this->sendCommand("?V\r", "");
	bool rval = this->responseParser<int>(3,&dV,&bV,&v5);
	driverVolt = dV/10.; //returned from motor controller in decivolts
	batteryVolt = bV/10.; //returned from motor controller in decivolts
	v5out = v5/1000.; //returned from motor controller in millivolts
	return rval;
}

bool MotorController::loadEEPROMSettings() {
	return this->sendCommand("%EELD\r", "+");
}

bool MotorController::failSafeReset() {
	//return this->sendCommand("%EERST 321654987\r");
	return false;
}

bool MotorController::saveConfigSettings() {
	LOG_MOTOR(FATAL) << "MotorController: saveConfigSettings" << endl;//TODO saveConfigSettings
	return false;
}

bool MotorController::reset() {
	stringstream stream;
	stream << "%RESET " << MotorController::MOTOR_SECRET_KEY << "\r";
	bool rval = this->sendCommand(stream.str(), "Starting...");
	this->serialPort.purgeBuffers();
	return rval;
}

bool MotorController::setTime( int hours, int minutes, int seconds) {
	stringstream stream;
	stream << "%STIME " << hours << ":" << minutes << ":" << seconds << "\r";
	return this->sendCommand(stream.str(), "+");
}

bool MotorController::setCommandLinearity(int linearity ) {
	LOG_MOTOR(FATAL) << "MotorController: setCommandLinearity" << endl;//TODO setCommandLinearity
	return false;
}

bool MotorController::setCommandPriority( int signal, int priority ) {
	LOG_MOTOR(FATAL) << "MotorController: setCommandPriority" << endl;//TODO setCommandPriority
	return false;
}

bool MotorController::setSerialWatchDogTimer( int time ) {
	LOG_MOTOR(FATAL) << "MotorController: setSerialWatchDogTimer" << endl;//TODO setSerialWatchDogTimer

	return false;
}

bool MotorController::setEncoderPPR( int ppr ) {
	LOG_MOTOR(FATAL) << "MotorController: setEncoderPPR" << endl;//TODO setEncoderPPR
	return false;
}

bool MotorController::setOverVoltageLimit( double limit ) {
	LOG_MOTOR(FATAL) << "MotorController: setOverVoltageLimit" << endl;//TODO setOverVoltageLimit
	return false;
}

bool MotorController::setUnderVoltageLimit( double limit ) {
	LOG_MOTOR(FATAL) << "MotorController: setUnderVoltageLimit" << endl;//TODO setUnderVoltageLimit
	return false;
}

bool MotorController::setShortCircuitDetectionThreshold( int value ) {
	LOG_MOTOR(FATAL) << "MotorController: setShortCircuitDetectionThreshold" << endl;//TODO setShortCircuitDetectionThreshold
	return false;
}

bool MotorController::getMotorLimits( int & m1Max, int & m2Max, int & m1Min, int & m2Min ) {
	m1Max = this->motor1SpeedMax;
	m2Max = this->motor2SpeedMax;
	m1Min = this->motor1SpeedMin;
	m2Min = this->motor2SpeedMin;
	return true;
}

bool MotorController::setLoopMode( MotorChannel channel, std::string & loopMode ) {

	bool rval = true;
	LOG_MOTOR(DEBUG4) << "Setting motor controller to " << loopMode << endl;

	if( "CLOSED_POSITION" == loopMode ) {
		rval = rval && this->sendCommand("^MMOD 1 3", "+");
		rval = rval && this->sendCommand("^MMOD 2 3", "+");
	} else if( "CLOSED_LOOP" ==  loopMode ) {
		rval = rval && this->sendCommand("^MMOD 1 2", "+");
		rval = rval && this->sendCommand("^MMOD 2 2", "+");
	} else if("OPEN_LOOP" == loopMode ){
		rval = rval && this->sendCommand("^MMOD 1 1", "+");
		rval = rval && this->sendCommand("^MMOD 2 1", "+");
	} else {
		LOG_MOTOR(FATAL) << "Invalid Loop Mode" << endl;
		this->emergencyStop();
	}

	return rval;
}

bool MotorController::assertValidMotorRange( int max, int min, int value ) const {
	return ( value > min && value < max );
}

bool MotorController::assertValidVoltage() {
	this->sendCommand("?V\n\r", "");//TODO parse the voltage command and check it is safe
	LOG_MOTOR(FATAL) << "MotorController: assertValidVoltage not implemented" << endl;
	return false;
}

bool MotorController::sendCommand( std::string command, std::string expectedResponse ) {
	bool rval = false;

	bool timeout;
	if( this->serialPort.isOpen() ) {
		LOG_MOTOR(DEBUG4) << "Purging Motor Controller Serial Port buffers" << endl;
		this->serialPort.purgeBuffers();
		LOG_MOTOR(DEBUG4) << "Sending to motor controller: " << command << endl;
		this->serialPort.Write(command.c_str(), command.length());
		this->responseBuffer = this->serialPort.ReadString(1000, &timeout, "\r" );

		if(timeout) {
			LOG_MOTOR(FATAL) << "Command: " << command << " timed out" << endl;
		}

		if( 0 == this->responseBuffer.length() ) {
			LOG_MOTOR(FATAL) << "Received a zero length response from Motor Controller" << endl;
		}

		LOG_MOTOR(DEBUG4) << "Response: " << this->responseBuffer << endl;
	}
	if( this->echoEnabled ) {

	} else {

	}
	rval = true;//TODO change this to detect verification of the command
	return rval;
}

void MotorController::enableSerialEcho() {
	LOG_MOTOR(DEBUG4) << "Enabling Echo on motor controller" << endl;
	this->sendCommand("^ECHOF 0\n\r", "+");
}

void MotorController::disableSerialEcho() {
	LOG_MOTOR(DEBUG4) << "Disabling Echo on motor controller" << endl;
	this->sendCommand("^ECHOF 1\n\r", "+");
}

bool MotorController::getFaultFlags() {
	LOG_MOTOR(FATAL) << "MotorController: getFaultFlags" << endl;//TODO getFaultFlags
	return false;
}

bool MotorController::getStatusFlags() {
	LOG_MOTOR(FATAL) << "MotorController: getStatusFlags not implemented" << endl;//TODO getStatusFlags
	return false;
}

bool MotorController::getControlUnitType() {
	LOG_MOTOR(FATAL) << "MotorController: getControlUnitType not implemented" << endl;//TODO getControlUnitType
	return false;
}

bool MotorController::clearBufferHistory() {
	LOG_MOTOR(FATAL) << "MotorController: clear Buffer History not implemented" << endl;//TODO clearBufferHistory
	return false;
}
