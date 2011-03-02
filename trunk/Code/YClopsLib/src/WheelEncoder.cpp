/**
 * @file WheelEncoder.cpp
 * @date Feb 24, 2011
 * @author Thomas Eldon Allred
 */

#include "WheelEncoder.h"
#include "MotorController.h"
#include "logging.h"

#include <iostream>

using namespace std;

WheelEncoder::WheelEncoder(): leftCount(0), rightCount(0), leftCountAbsolute(0), rightCountAbsolute(0) { }

void WheelEncoder::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

}

void WheelEncoder::init() {
	MotorController::instance()->setEncoderCounter(MotorController::BothChannels,0);
}

void WheelEncoder::sensorProcess() {
	int tmp1, tmp2;
	LOG_ENCODER(DEBUG4) << "Getting Absolute Encoder Values" << endl;
	MotorController::instance()->getAbsoluteEncoderCount(this->leftCountAbsolute, this->rightCountAbsolute);
	LOG_ENCODER(DEBUG4) << "Getting Relative Encoder Values" << endl;
	MotorController::instance()->getRelativeEncoderCount(tmp1,tmp2);
	this->leftCount += tmp1;
	this->rightCount += tmp2;
}

SensorData * WheelEncoder::getData() {
	EncoderData* data = new EncoderData(this->leftCount, this->rightCount, this->leftCountAbsolute, this->rightCountAbsolute);
	this->leftCount = 0;
	this->rightCount = 0;
	return data;
}

void WheelEncoder::dumpData( std::ostream & out ) const {
	out << "*******************************" << endl;
	out << "Left Encoder value: " << this->leftCount << endl;
	out << "Right Encoder value: " << this->rightCount << endl;
	out << "Left Absolute value: " << this->leftCountAbsolute << endl;
	out << "Right Absolute value: " << this->rightCountAbsolute << endl;
	out << "*******************************" << endl;
}

WheelEncoder::~WheelEncoder() { }
