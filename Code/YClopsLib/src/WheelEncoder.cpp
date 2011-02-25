/*
 * WheelEncoder.cpp
 *
 *  Created on: Feb 24, 2011
 *      Author: tallred3
 */

#include "WheelEncoder.h"
#include "MotorController.h"
#include "logging.h"

#include <iostream>

using namespace std;

WheelEncoder::WheelEncoder() { }

void WheelEncoder::loadConfiguration(mrpt::utils::CConfigFileBase & config, std::string & sectionName) {
	LOG(FATAL) << "WheelEncoder: loadConfiguration not implemented" << endl;
}

void WheelEncoder::init() {
	LOG(FATAL) << "WheelEncoder: init not implemented" << endl;
}

void WheelEncoder::sensorProcess() {
	LOG(FATAL) << "WheelEncoder: sensorProcess not implemented" << endl;
}

SensorData * WheelEncoder::getData() {
	LOG(FATAL) << "WheelEncoder: getData not implemented" << endl;
	return new EncoderData();
}

void WheelEncoder::dumpData( std::ostream & out ) {
	LOG(FATAL) << "WheelEncoder: dumpData not implemented" << endl;
}

WheelEncoder::~WheelEncoder() { }
