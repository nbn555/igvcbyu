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
#include "logging.h"

#include "MotorController.h"
#include "YClopsConfiguration.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

MotorController * MotorController::mc = NULL;
const std::string MotorController::MOTOR_BAD_COMMAND = "-";

MotorController * MotorController::instance() {
	if( NULL == MotorController::mc ) {
		LOG_MOTOR(DEBUG2) << "Initializing Motor Controller" << endl;
		MotorController::mc = new MotorController();
	}
	return MotorController::mc;
}

MotorController::MotorController():
		echoEnabled(YClopsConfiguration::instance().read_bool("MOTOR", "ECHO_ENABLED", true)),
		motor1Speed(0), motor2Speed(0),
		motor1SpeedMax(YClopsConfiguration::instance().read_int("MOTOR", "MAX_FORWARD_LEFT", 0) ),  //If the config file isn't correct we don't want to be able to do anything for safety reasons
		motor2SpeedMax(YClopsConfiguration::instance().read_int("MOTOR", "MAX_FORWARD_RIGHT", 0 ) ),
		motor1SpeedMin(YClopsConfiguration::instance().read_int("MOTOR", "MAX_REVERSE_LEFT", 0 ) ),
		motor2SpeedMin(YClopsConfiguration::instance().read_int("MOTOR", "MAX_REVERSE_RIGHT", 0) ),
		permissiveMode(YClopsConfiguration::instance().read_bool("MOTOR", "PERMISSIVE", false))
{

	string portName = YClopsConfiguration::instance().read_string("MOTOR", "COM_port_LIN", "/dev/ttyS1" );

	LOG_MOTOR(DEBUG4) << "Connecting to Motor Controller on port: " << portName << endl;
	this->serialPort.open(portName);


	this->serialPort.setConfig(115200, 0, 8, 1, false);//Non configurable port setting see the roboteq manual

	if( -1 == sem_init( &(this->serialPortSem), 0, 1 ) ) {
		LOG_MOTOR(FATAL) << "Motor Controller Serial Port Semaphore failed to initialize" << endl;
	}

//	if( !this->reset() ) {
//		LOG_MOTOR(FATAL) << "Motor Controller Failed to successfully reset" << endl;
//	}

	if(echoEnabled) {
		this->enableSerialEcho(); //enable the serial to echo the commands back used for error checking
	}else {
		this->disableSerialEcho();//disable serial port echo
	}

	if(!this->clearBufferHistory()){
		LOG_MOTOR(WARNING) << "MotorController Clear buffer history failed" << endl;
	}

	this->mixingMode = YClopsConfiguration::instance().read_int("MOTOR", "MIXING_MODE", 0);
	if(!this->setMixingMode(this->mixingMode)) {
		LOG_MOTOR(WARNING) << "MotorController Mixing Mode failed" << endl;
	}

	this->operatingMode = YClopsConfiguration::instance().read_int( "MOTOR", "OPERATING_MODE", 1);//Default to open loop speed for safety
	if(!this->setOperatingMode(this->mixingMode)) {
		LOG_MOTOR(WARNING) << "MotorController Set Operating Mode failed" << endl;
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
		parser << "!M " << this->motor1Speed << " " << this->motor2Speed << "\r";
		this->sendCommand(parser.str(), "+", NULL, this->permissiveMode);
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
	LOG_MOTOR(DEBUG3) << "Setting emergency stop" << endl;
	return this->sendCommand("!EX\r", "+", NULL, this->permissiveMode);
}

bool MotorController::clearEmergencyStop() {
	LOG_MOTOR(DEBUG3) << "Clearing emergency stop" << endl;
	return this->sendCommand("!MG\r", "+", NULL, this->permissiveMode);
}

bool MotorController::setSpeed( MotorChannel channel, int value ) {
	LOG_MOTOR(DEBUG3) << "MotorController: setSpeed" << endl;
	bool rval = false;
	switch(channel) {
	case Channel1:
		value = value > motor1SpeedMax ? motor1SpeedMax : value;
		value = value < motor1SpeedMin ? motor1SpeedMin : value;
		this->motor1Speed = value;
		rval = true;
		break;
	case Channel2:
		value = value > motor2SpeedMax ? motor2SpeedMax : value;
		value = value < motor2SpeedMin ? motor2SpeedMin : value;
		this->motor2Speed = value;
		rval = true;
		break;
	default:
		LOG_MOTOR(FATAL) << "Invalid Motor Channel" << endl;
		break;
	}

	return rval;
}

bool MotorController::setEncoderCounter( MotorChannel channel, int value ) {
	LOG_MOTOR(DEBUG3) << "MotorController: setEncoderCounter" << endl;
	bool rval = false;
	stringstream parser;

	switch(channel) {
	case Channel1:
		parser << "!C 1 " << value << '\r';
		rval = this->sendCommand(parser.str(),"+", NULL, this->permissiveMode);
		break;
	case Channel2:
		parser << "!C 2 " << value << '\r';
		rval = this->sendCommand(parser.str(),"+", NULL, this->permissiveMode);
		break;
	case BothChannels:
		parser << "!C 1 " << value << '\r';
		rval = this->sendCommand(parser.str(),"+", NULL, this->permissiveMode);
		parser.str("");
		parser << "!C 2 " << value << '\r';
		rval = rval && this->sendCommand(parser.str(),"+", NULL, this->permissiveMode);
		break;
	default:
		LOG_MOTOR(ERROR) << "Invalid motor channel for encoder set value" << endl;
	}

	return rval;
}

bool MotorController::setEncoderUsage( int value ) {
	LOG_MOTOR(DEBUG3) << "MotorController: setEncoderUsage" << endl;
	bool rval = false;
	stringstream parser;

	parser << "^EMOD 1 17\r";
	rval = this->sendCommand(parser.str(), "+", NULL, this->permissiveMode);
	parser.str("");
	parser << "^EMOD 2 33\r";
	rval = rval && this->sendCommand(parser.str(), "+", NULL, this->permissiveMode);

	return rval;
}

bool MotorController::setMixingMode( int value ) {
	LOG_MOTOR(DEBUG3) << "MotorController: setMixingMode" << endl;
	bool rval = false;

	this->currentMixingMode = value;

	stringstream parser;
	parser << "^MXMD " << value << '\r';
	rval = this->sendCommand(parser.str(), "+", NULL, true);
	//Need to run this command in permissive because I think there is a bug in the
	//Motor controller where it sometimes returns /r+/r instead of +/r

	return rval;
}

bool MotorController::restoreMixingMode() {
	LOG_MOTOR(DEBUG3) << "MotorController: restoreMixingMode" << endl;
	return this->setMixingMode(this->mixingMode);
}

bool MotorController::setOperatingMode( int value ) {
	LOG_MOTOR(DEBUG3) << "MotorController: setOperatingMode" << endl;
	bool rval = false;

	if( (1 != value) && (2 != value) && (3 != value) ) LOG_MOTOR(ERROR) << "MotorController: Invalid Operating Mode Value:" << value << endl;

	this->currentOperatingMode = value;

	stringstream parser;
	parser << "^MMOD 1 " << value << '\r';
	rval = this->sendCommand(parser.str(), "+", NULL, this->permissiveMode);

	parser.str("");

	parser << "^MMOD 2 " << value << '\r';
	rval = rval && this->sendCommand(parser.str(), "+", NULL, this->permissiveMode);

	return rval;
}
bool MotorController::restoreOperatingMode() {
	LOG_MOTOR(DEBUG3) << "MotorController: restoreOperatingMode" << endl;
	return this->setOperatingMode(this->operatingMode);
}

bool MotorController::getMotorAmps( double & motor1Amps, double & motor2Amps ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getMotorAmps" << endl;
	int channel1;
	int channel2;

	string response;
	bool rval = this->sendCommand("?A\r", "A=", &response, this->permissiveMode);
	rval = rval && this->responseParser( response, 2, &channel1, &channel2);

	motor1Amps = channel1/10.; //The value returned is in deciamps
	motor2Amps = channel2/10.;

	return rval;
}

bool MotorController::getAnalogInputs( int input, int & value ) {
	LOG_MOTOR(FATAL) << "MotorController: getAnalogInputs" << endl;//TODO getAnalogInputs
	return false;
}

bool MotorController::getBatteryAmps( double & motor1Amps, double & motor2Amps ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getBatteryAmps" << endl;
	int channel1;
	int channel2;

	string response;
	bool rval = this->sendCommand("?BA\r", "", &response, this->permissiveMode);
	rval = rval && this->responseParser(response, 2, &channel1, &channel2);

	motor1Amps = channel1/10.; //The value returned is in deciamps converting to amps
	motor2Amps = channel2/10.;

	return rval;
}

bool MotorController::getAbsoluteEncoderCount(int & ch1, int & ch2 ) {
	bool rval = false;
	LOG_MOTOR(DEBUG3) << "MotorController: GetAbsoluteEncoderCount" << endl;
	LOG_MOTOR(DEBUG4) << "absolute encoder count pre ch1 = " << ch1 << " ch2 = " << ch2 << endl;
	string response;
	rval = this->sendCommand("?C\r", "C=", &response, this->permissiveMode);
	rval = rval && this->responseParser(response, 2, &ch1, &ch2);
	LOG_MOTOR(DEBUG4) << "absolute encoder count Got ch1 = " << ch1 << " ch2 = " << ch2 << endl;
	return rval;
}

bool MotorController::getRelativeEncoderCount( int & ch1, int & ch2 ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getRelativeEncoderCount" << endl;
	string response;
	this->sendCommand("?CR\r", "CR=", &response, this->permissiveMode);
	return this->responseParser(response, 2, &ch1, &ch2);
}

bool MotorController::getEncoderSpeeds( int & speed1, int & speed2 ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getEncoderSpeed" << endl;
	bool rval = false;
	string response;
	rval = this->sendCommand("?S\r","S=", &response, this->permissiveMode);
	rval = rval && this->responseParser( response, 2, &speed1, &speed2 );
	return rval;
}

bool MotorController::getTemperature( int & ch1, int & ch2, int & ic ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getTemperature" << endl;
	string response;
	this->sendCommand("?T 1\r", "T=", &response, this->permissiveMode);
	return this->responseParser( response, 3,&ic,&ch1,&ch2);
}

bool MotorController::getTime( int & hours, int & minutes, int & seconds ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getTime" << endl;
	string response;
	this->sendCommand("?TM\r", "TM=", &response, this->permissiveMode);
	return this->responseParser(response, 3,&hours,&minutes,&seconds);
}

bool MotorController::getVoltages( double & driverVolt, double & batteryVolt, double & v5out ) {
	LOG_MOTOR(DEBUG3) << "MotorController: getVoltages" << endl;
	int dV;
	int bV;
	int v5;

	string response;
	this->sendCommand("?V\r", "V=", &response, this->permissiveMode);
	bool rval = this->responseParser(response, 3,&dV,&bV,&v5);
	driverVolt = dV/10.; //returned from motor controller in decivolts
	batteryVolt = bV/10.; //returned from motor controller in decivolts
	v5out = v5/1000.; //returned from motor controller in millivolts
	return rval;
}

bool MotorController::loadEEPROMSettings() {
	LOG_MOTOR(DEBUG3) << "MotorController: Load EEPROM settings" << endl;
	//TODO check the timing on this function
	return this->sendCommand("%EELD\r", "+", NULL, this->permissiveMode);
}

bool MotorController::failSafeReset() {
	stringstream stream;
	stream << "%EERST " << MotorController::MOTOR_SECRET_KEY << "\r";
	bool rval = this->sendCommand(stream.str(), "Starting...", NULL, this->permissiveMode);
	sleep(3);
	this->serialPort.purgeBuffers();
	return rval;
}

bool MotorController::saveConfigSettings() {
	LOG_MOTOR(FATAL) << "MotorController: saveConfigSettings" << endl;//TODO saveConfigSettings
	return false;
}

bool MotorController::reset() {
	stringstream stream;
	stream << "%RESET " << MotorController::MOTOR_SECRET_KEY << "\r";
	bool rval = this->sendCommand(stream.str(), "Starting...", NULL, this->permissiveMode);
	sleep(3);//need to allow the controller to power cycle
	this->serialPort.purgeBuffers();
	return rval;
}

bool MotorController::setTime( int hours, int minutes, int seconds) {
	LOG_MOTOR(DEBUG3) << "Sending Time to MotorController" << endl;
	stringstream stream;
	stream << "%STIME " << hours << ":" << minutes << ":" << seconds << "\r";
	return this->sendCommand(stream.str(), "+", NULL, this->permissiveMode);
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
	LOG_MOTOR(DEBUG3) << "MotorController: setEncoderPPR" << endl;
	bool rval = false;
	stringstream stream;
	stream << "^EPPR 1 " << ppr << "\r";
	rval = this->sendCommand(stream.str(), "+", NULL, this->permissiveMode);
	stream.str("");
	stream << "^EPPR 2 " << ppr << "\r";
	rval = rval && this->sendCommand(stream.str(),"+", NULL, this->permissiveMode);
	return rval;
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
		rval = rval && this->sendCommand("^MMOD 1 3\r", "+", NULL, this->permissiveMode);
		rval = rval && this->sendCommand("^MMOD 2 3\r", "+", NULL, this->permissiveMode);
	} else if( "CLOSED_LOOP" ==  loopMode ) {
		rval = rval && this->sendCommand("^MMOD 1 2\r", "+", NULL, this->permissiveMode);
		rval = rval && this->sendCommand("^MMOD 2 2\r", "+", NULL, this->permissiveMode);
	} else if("OPEN_LOOP" == loopMode ){
		rval = rval && this->sendCommand("^MMOD 1 1\r", "+", NULL, this->permissiveMode);
		rval = rval && this->sendCommand("^MMOD 2 1\r", "+", NULL, this->permissiveMode);
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
	this->sendCommand("?V\r", "", NULL, this->permissiveMode);//TODO parse the voltage command and check it is safe
	LOG_MOTOR(FATAL) << "MotorController: assertValidVoltage not implemented" << endl;
	return false;
}

bool MotorController::sendCommand( const std::string command, const std::string expectedResponse, std::string * response, bool permissive ) {
	bool rval = false;

	string portResponse("");
	bool timeout = false;

	if( this->serialPort.isOpen() ) {

		LOG_MOTOR(DEBUG4) << "Locking the serial port semaphore" << endl;
		sem_wait(&(this->serialPortSem));

		LOG_MOTOR(DEBUG4) << "Purging Motor Controller Serial Port buffers" << endl;
		this->serialPort.purgeBuffers();

		LOG_MOTOR(DEBUG3) << "Sending to motor controller: " << command << endl;
		this->serialPort.Write(command.c_str(), command.length());

		portResponse = this->serialPort.ReadString(1000, &timeout, "\r" );
		LOG_MOTOR(DEBUG3) << "Response: " << portResponse << endl;

		LOG_MOTOR(DEBUG4) << "Purging Motor Controller Serial Port buffers" << endl;
		this->serialPort.purgeBuffers();

		sem_post(&(this->serialPortSem));
		LOG_MOTOR(DEBUG4) << "Unlocked Serial Port Semaphore" << endl;

		if( !permissive ) {

			if( 0 != expectedResponse.length() ) {

				if(timeout) {
					LOG_MOTOR(FATAL) << "Command: time out " << command << endl;
				}

				if( 0 == portResponse.length() ) {
					LOG_MOTOR(FATAL) << "Received a zero length response from Motor Controller" << endl;
				}

				if( expectedResponse == portResponse.substr(0,expectedResponse.length()) ) {
					portResponse = portResponse.substr(expectedResponse.length());
					rval = true;
				} else if( MotorController::MOTOR_BAD_COMMAND == portResponse.substr(0,MotorController::MOTOR_BAD_COMMAND.length())) {
					LOG_MOTOR(ERROR) << "Misunderstood command sent to MotorController" << endl;
				} else {
					LOG_MOTOR(FATAL) << "Unanticipated response received from MotorController expected: " << expectedResponse << " but got " << portResponse << endl;
				}
			} else rval = true;
		}
	}

	if( NULL != response ) {
		*response = portResponse;
	} else rval = true;

	LOG_MOTOR(DEBUG4) << "Command " << (rval?"success":"failure") << endl;
	return rval;
}

void MotorController::enableSerialEcho() {
	LOG_MOTOR(DEBUG4) << "Enabling Echo on motor controller" << endl;
	this->sendCommand("^ECHOF 0\r", "+", NULL, true);
}

void MotorController::disableSerialEcho() {
	LOG_MOTOR(DEBUG4) << "Disabling Echo on motor controller" << endl;
	this->sendCommand("^ECHOF 1\r", "+", NULL, true);
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
	LOG_MOTOR(DEBUG4) << "MotorController: clear Buffer History" << endl;
	return this->sendCommand("# C\r", "", NULL, true);
}
