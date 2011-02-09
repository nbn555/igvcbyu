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

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace boost;

Compass::Compass( bool degrees, const int bufferLength ): degrees(degrees), yaw(0), pitch(0), roll(0), yawStatus(""), pitchStatus(""), rollStatus("") {
}

Compass::Compass( std::string iniFile ): degrees(0), yaw(0), pitch(0), roll(0), yawStatus(""), pitchStatus(""), rollStatus("") {

	CConfigFile config(iniFile);
	this->init(config);
}

Compass::Compass( mrpt::utils::CConfigFile & config ): degrees(0), yaw(0), pitch(0), roll(0), yawStatus(""), pitchStatus(""), rollStatus("") {
	this->init(config);
}
Compass::~Compass() {

}

void Compass::init( CConfigFile & config ) {

	this->loadConfig(config,"COMPASS");
	this->loadConfig_sensorSpecific(config,"COMPASS");
	this->initialize();
	this->doProcess();

}
void Compass::doProcess() {
	string data;

	bool booleanness = true;
	data = this->serialPort.ReadString(100,&booleanness, "\n\r");
	this->parseResponse(data);

}

void Compass::dumpData( std::ostream & out ) {
	out << this->yaw << ", " << this->pitch << ", " << this->roll << "Valid: " << this->yawStatus << this->pitchStatus << this->rollStatus << endl;
}

void Compass::loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase& config, const std::string& sectionName ) {

	string value = config.read_string(sectionName, "COM_port_LIN", "/dev/ttyS1" );
	this->serialPort.open(value);

	int baudRate = config.read_int(sectionName, "baudRate", 19200 );

	this->serialPort.setConfig( baudRate, 0, 8, 1 );

	string writeValue = "#FA0.3=1*26\n\r";//Command to turn on the compass
	this->serialPort.WriteBuffer(writeValue.c_str(), writeValue.length());
	string readValue;
	bool to = false;
	readValue = this->serialPort.ReadString(1000,&to,"\n\r");
	this->degrees = config.read_bool("COMPASS","degrees",false);

}

//TODO make this not return null.  I couldn't get it to work because it wants a pointer to the class constructor but the c++ standard says you cant do that.
const mrpt::hwdrivers::TSensorClassId* Compass::GetRuntimeClass() const {
	return NULL;//static_cast<	const mrpt::hwdrivers::TSensorClassId*>( & Compass::classCompass );
}

void Compass::parseResponse( const std::string& data ) {

	string buffer = data;
	string header = "";

	boost::replace_all( buffer, ",", " " );//replace the commas with spaces so the stringstream parser will parse how we want
	stringstream s(buffer);

	s >> header;
	s >> this->yaw;
	s >> this->yawStatus;
	s >> this->pitch;
	s >> this->pitchStatus;
	s >> this->roll;
	s >> this->rollStatus;

}

void Compass::reset() {
	this->yaw = 0;
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
