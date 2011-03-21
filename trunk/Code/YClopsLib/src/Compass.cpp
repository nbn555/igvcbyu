/*
 * Compass.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: igvcbyu
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

Compass::Compass(): degrees(false), yaw(0), pitch(0), roll(0), yawStatus(""), pitchStatus(""), rollStatus(""), serialPort() {

}

void Compass::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName ) {

	string value = config.read_string(sectionName, "COM_port_LIN", "/dev/ttyS1" );
	LOG_COMPASS(DEBUG3) << "Using " << value << " port for compass" << endl;

	this->serialPort.open(value);

	int baudRate = config.read_int(sectionName, "baudRate", 19200 );

	LOG_COMPASS(DEBUG3) << "Using " << baudRate << " for compass baudrate" << endl;

	this->serialPort.setConfig( baudRate, 0, 8, 1 );

	this->offset = config.read_int(sectionName, "offset", 0 );

	this->degrees = config.read_bool(sectionName,"degrees",false);

	LOG_COMPASS(DEBUG3) << "Using degrees mode: " << (this->degrees ? "TRUE" : "FALSE") << endl;

}

void Compass::init() {

	string writeValue = "#FA0.3=1*26\n\r";//Command to turn on the compass

	LOG_COMPASS(DEBUG3) << "Sending: " << writeValue << endl;

	this->serialPort.WriteBuffer(writeValue.c_str(), writeValue.length());

	string readValue;

	bool to = false;

	readValue = this->serialPort.ReadString(1000,&to,"\n\r");

	LOG_COMPASS(DEBUG3) << "Read: " << readValue << endl;

}

Compass::~Compass() {

}

void Compass::sensorProcess() {
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
	s >> preYaw;
	this->yaw = CompensateYaw(preYaw);
	s >> this->yawStatus;
	s >> this->pitch;
	s >> this->pitchStatus;
	s >> this->roll;
	s >> this->rollStatus;

}

void Compass::reset() {
	this->yaw = 999;
	this->pitch = 0;
	this->roll = 0;
	this->yawStatus = "";
	this->pitchStatus = "";
	this->rollStatus = "";
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

	if( 0 <= high && high <= 9 ) high += 0x30;
	else if( 10 <= high && high <= 15 ) high += 0x40;

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
	if (deg == 0) return deg; // is defaults to zero if not found
	if (deg < 200)
		deg = deg + 34*sin(.93*piToD*deg - 0.15) + (double)this->offset;
	else
		deg = deg + 18 * sin(deg*piToD - .3) + (double)this->offset;

	return deg;
}
