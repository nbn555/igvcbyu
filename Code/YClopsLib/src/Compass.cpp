/**
 * @file Compass.cpp
 * @date Nov 4, 2010
 * @author igvcbyu
 */

#include <cassert>

#include <boost/algorithm/string.hpp>
#include <mrpt/utils/CConfigFile.h>
#include "Compass.h"
#include "logging.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace boost;


std::string Compass::setParameterResponse = "#!0000*21";

IMPLEMENTS_GENERIC_SENSOR(Compass, mrpt::hwdrivers);

Compass::Compass(): degrees(false), prevYawDeg(0), yaw(0), pitch(0), roll(0), deviation(0), variation(0), yawStatus(""), pitchStatus(""), rollStatus(""), serialPort()
{
	mrpt::hwdrivers::CGenericSensor::registerClass(SENSOR_CLASS_ID(Compass));
}
void Compass::loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName ) {

	string value = config.read_string(sectionName, "COM_port_LIN", "/dev/ttyS1" );
	LOG_COMPASS(DEBUG3) << "Using " << value << " port for compass" << endl;

	this->serialPort.open(value);

	int baudRate = config.read_int(sectionName, "baudRate", 19200 );

	LOG_COMPASS(DEBUG3) << "Using " << baudRate << " for compass baudrate" << endl;

	this->serialPort.setConfig( baudRate, 0, 8, 1 );

	this->deviation = config.read_double( sectionName, "deviation", 0 );

	this->variation = config.read_double( sectionName, "variation", 0 );

	this->offset = config.read_int(sectionName, "offset", 0 );

	this->degrees = config.read_bool(sectionName,"degrees",false);

	LOG_COMPASS(DEBUG3) << "Using degrees mode: " << (this->degrees ? "TRUE" : "FALSE") << endl;

}

void Compass::initialize() {

	LOG_COMPASS(DEBUG3) << "Sending command to turn off commands sending from the compass" << endl;
	if( !this->sendCommand( string("#FA0.3=0*27\r\n"), string("#!0000*21")) ) {
		LOG_COMPASS(FATAL) << "Error response coming from compass" << endl;
	}


	LOG_COMPASS(DEBUG3) << "Sending command to set to Degrees mode" << endl;
	if( !this->sendCommand( string("#FA0.4=1*21\r\n"), string("#!0000*21") ) ) {
		LOG_COMPASS(FATAL) << "Error response coming from compass" << endl;
	}

	LOG_COMPASS(DEBUG3) << "Sending command to set to Decimal mode" << endl;
	if( !this->sendCommand( string("#FA0.5=1*20\r\n"), string("#!0000*21") ) ) {
		LOG_COMPASS(FATAL) << "Error response coming from compass" << endl;
	}

	LOG_COMPASS(DEBUG3) << "Sending command to set sampling rate to 13.75Hz" << endl;
	if( !this->sendCommand( string("#BA6=1*39\r\n"), string("#!0000*21") ) ) {
		LOG_COMPASS(FATAL) << "Error response coming from compass" << endl;
	}

	LOG_COMPASS(DEBUG3) << "Sending command to set the compass variation to " << this->variation << endl;
	if( !this->setVariation( this->variation ) ) {
		LOG_COMPASS(FATAL) << "Error sending variation to compass" << endl;
	}

	LOG_COMPASS(DEBUG3) << "Sending command to set the compass deviation to " << this->variation << endl;
	if( !this->setDeviation( this->deviation ) ) {
		LOG_COMPASS(FATAL) << "Error sending deviation to compass" << endl;
	}

	LOG_COMPASS(DEBUG3) << "Sending command to turn on the compass sending data" << endl;
	if( !this->sendCommand( string("#FA0.3=1*26\r\n"), string("#!0000*21")) ) {
		LOG_COMPASS(FATAL) << "Error response coming from compass" << endl;
	}

}

Compass::~Compass() {
	LOG_COMPASS(DEBUG3) << "Turning off the compass in compass destructor" << endl;
	this->sendCommand( string("#FA0.3=0*27\r\n"), string("#!0000*21"));//!<TODO make the compass parser check if the return string was in the middle of another output
	//Don't really care if it was put back into its turned off state for now
}

bool Compass::setDeviation( double deviation ) {
	string header = "#IE2=";
	stringstream s;
	s.precision(1);
	s << deviation;
	string deviationString = s.str();
	s.str("");
	s << header << deviationString << "*";
	s << this->computeChecksum(s.str()) << "\r\n";

	stringstream expecteds;
	expecteds << "#" << deviationString << "*";
	expecteds << this->computeChecksum(expecteds.str()) << "\r\n";

	return this->sendCommand( s.str(), "#!0000*21" );

}

bool Compass::setVariation( double variation ) {
	//!<TODO refactor setDeviation with setVariation
	string header = "#IE4=";
	stringstream s;
	s.precision(1);
	s << deviation;
	string deviationString = s.str();
	s.str("");
	s << header << deviationString << "*";
	s << this->computeChecksum(s.str()) << "\r\n";

	stringstream expecteds;
	expecteds << "#" << deviationString << "*";
	expecteds << this->computeChecksum(expecteds.str()) << "\r\n";

	return this->sendCommand( s.str(), "#!0000*21" );

}

bool Compass::setCalibrationMode() {
	//!<TODO refactor serial port communication protocalls
	string header = "#F33.4=0";
	stringstream s;

	s << header << "*";
	s << this->computeChecksum(s.str()) << "\r\n";

	return this->sendCommand( s.str(), "#!0000*21" );

}

bool Compass::saveCalibrationResults() {
	string header = "#F33.4=1";
	stringstream s;

	s << header << "*";
	s << this->computeChecksum(s.str()) << "\r\n";

	return this->sendCommand( s.str(), "#!0000*21" );

}

double Compass::getCalibrationProgress() {
	//!<todo parse the calibration response
	const double MAX_ITERATIONS = 275; //see hmr3000 datasheet
	string header = "#I26C?";
	stringstream s;

	s << header << "*";
	s << this->computeChecksum(s.str()) << "\r\n";

	return this->sendCommand( s.str(), "#!0000*21" );
}

void Compass::doProcess() {
	string data;

	bool booleanness = true;
	data = this->serialPort.ReadString(100,&booleanness, "\n\r");
	this->serialPort.purgeBuffers();
	this->parseResponse(data);

}

SensorData * Compass::getData() {
	return new CompassData(this->getYaw(), this->getPitch(), this->getRoll(), "N" == this->yawStatus.substr(0,1), "N" == this->pitchStatus.substr(0,1), "N" == this->rollStatus.substr(0,1), this->degrees );
}

void Compass::dumpData( std::ostream & out ) const {
	out << "************" << endl;
	out << "Compass" << endl;
	out << this->yaw << ", " << this->pitch << ", " << this->roll << " Valid: " << this->yawStatus << this->pitchStatus << this->rollStatus << endl;
	out << "************" << endl;
}

void Compass::parseResponse( const std::string& data ) {

	string buffer = data;
	string header = "";
	//! preYaw for yaw compensation
	double preYaw = 0;

	boost::replace_all( buffer, ",", " " );//replace the commas with spaces so the stringstream parser will parse how we want
	stringstream s(buffer);

	s >> header;

	cout << header << endl;

	s >> preYaw;
	this->yaw = CompensateYaw(preYaw);
	s >> this->yawStatus;
	s >> this->pitch;
	s >> this->pitchStatus;
	s >> this->roll;
	s >> this->rollStatus;

}

std::string Compass::computeChecksum( const std::string & sentence ) {

	const int strLen = sentence.length();

	int i = 0, j = 0;
	for( i = 0; i < strLen && '$' != sentence.at(i) && '#' != sentence.at(i); ++i );
	if( strLen <= i ) return "";

	++i;
	for( j = i; j < strLen && '*' != sentence.at(j); ++j );
	if( strLen <= j ) return "";

	unsigned char checksum = 0;
	for( ; i != j; ++i ) checksum ^= sentence.at(i);

	unsigned char high = (checksum & 0xF0) >> 4;
	unsigned char low = checksum & 0x0F;

	//Converting high bits to their ascii representation
	if( 0 <= high && high <= 9 ) high += 0x30;
	else if( 10 <= high && high <= 15 ) high += 0x40;

	//Converting low bits to their ascii representation
	if( 0 <= low && low <= 9 ) low += 0x30;
	else if( 10 <= low && low <= 15 ) low += 0x40;

	stringstream stream;
	stream << high;
	stream << low;
	return stream.str();
}

double Compass::CompensateYaw(double deg) {
	double piToD = 3.14159265/180;
	//const double OFFSET = 7.4;
	if (deg == 0)
		deg = prevYawDeg; // returns zero sometimes, parse error?

	else if (deg < 200)
		deg = deg + 34*sin(.93*piToD*deg - 0.15) + (double)this->offset;
	else
		deg = deg + 18 * sin(deg*piToD - .3) + (double)this->offset;

	prevYawDeg = deg;

	return deg;
}

bool Compass::sendCommand( std::string command, std::string expectedResponse ) {

	LOG_COMPASS(DEBUG4) << "Sending: " << command << endl;

	this->serialPort.WriteBuffer(command.c_str(), command.length());

	bool to = false;

	string readValue = this->serialPort.ReadString(1000,&to,"\n\r");

	usleep(500000);

	LOG_COMPASS(DEBUG4) << "Read: " << readValue << endl;

	bool rval = readValue == expectedResponse;

	this->serialPort.purgeBuffers();

	if( !rval ) {
		LOG_COMPASS(ERROR) << "Expected: " << expectedResponse << "." << endl;
		LOG_COMPASS(ERROR) << "Recieved: " << readValue << "." << endl;
	}
	return rval;

}
