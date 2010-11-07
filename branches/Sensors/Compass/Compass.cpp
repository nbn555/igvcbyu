/*
 * Compass.cpp
 *
 *  Created on: Nov 4, 2010
 *      Author: igvcbyu
 */

#include <cassert>
#include <cstdlib>

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/types.h>
#include <boost/algorithm/string.hpp>

#include "Compass.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace boost;

Compass::Compass( const string & fileName, const int bufferLength ): yaw(0), pitch(0), roll(0) {
	CConfigFile config(fileName);

	this->loadConfig_sensorSpecific( config, "COMPASS" );

	this->initialize();
}

Compass::~Compass() {

}

void Compass::doProcess() {
	string data;

	bool booleanness = true;
	data = this->serialPort.ReadString(100,&booleanness, "\n\r");
	this->parseResponse(data);

}


void Compass::loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase& config, const std::string& sectionName ) {

	if( "COMPASS" == sectionName ) {
		vector_string sections;
		vector_string keys;

		config.getAllSections(sections);
		config.getAllKeys(sectionName,keys);

		string value = config.read_string(sectionName, "COM_port_LIN", value );
		this->serialPort.open(value);

		value = config.read_string(sectionName, "baudRate", value);

		this->serialPort.setConfig(atoi(value.c_str()), 0, 8, 1);
	}

}

//TODO make this not return null.  I couldn't get it to work because it wants a pointer to the class constructor but the c++ standard says you cant do that.
const mrpt::hwdrivers::TSensorClassId* Compass::GetRuntimeClass() const {
	return NULL;//static_cast<	const mrpt::hwdrivers::TSensorClassId*>( & Compass::classCompass );
}

void Compass::parseResponse( const std::string& data ) {

	string buffer = data;
	string header = "";

	this->reset();

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
