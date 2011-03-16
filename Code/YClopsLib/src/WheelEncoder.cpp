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

WheelEncoder::WheelEncoder(): leftCount(0), rightCount(0), leftCountAbsolute(0), rightCountAbsolute(0), leftEncoderSpeed(0), rightEncoderSpeed(0), encoderPPR(0) { }

void WheelEncoder::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	this->encoderPPR = config.read_int( sectionName, "ENCODER_PPR", 400 );

	this->leftCountAbsolute = config.read_int( sectionName, "ABSOLUTE_COUNTS_INIT", this->leftCountAbsolute);
	this->rightCountAbsolute = config.read_int( sectionName, "ABSOLUTE_COUNTS_INIT", this->rightCountAbsolute);

}

void WheelEncoder::init() {

	LOG_ENCODER(DEBUG4) << "Setting encoder usage to feedback" << endl;
	MotorController::instance()->setEncoderUsage();

	LOG_ENCODER(DEBUG4) << "Setting the EncoderPPR" << endl;
	MotorController::instance()->setEncoderPPR(this->encoderPPR);

	LOG_ENCODER(DEBUG4) << "Setting encoder settings" << endl;
	MotorController::instance()->setEncoderCounter(MotorController::BothChannels,this->leftCountAbsolute);
}

void WheelEncoder::sensorProcess() {
	int tmp1 = 0, tmp2 = 0;
	LOG_ENCODER(DEBUG4) << "Getting Absolute Encoder Values" << endl;
	MotorController::instance()->getAbsoluteEncoderCount(this->leftCountAbsolute, this->rightCountAbsolute);
	LOG_ENCODER(DEBUG4) << "Getting Relative Encoder Values" << endl;
	MotorController::instance()->getRelativeEncoderCount(tmp1,tmp2);
	this->leftCount += tmp1;
	this->rightCount += tmp2;
	LOG_ENCODER(DEBUG4) << "Getting Encoder Speed" << endl;
	MotorController::instance()->getEncoderSpeeds(this->leftEncoderSpeed, this->rightEncoderSpeed);

	LOG_ENCODER(DEBUG4) << "Encoder values in sensorProcess: LA " << this->leftCountAbsolute << " RA " << this->rightCountAbsolute << " L " << this->leftCount << " R " << this->rightCount << " LS " << this->leftEncoderSpeed << " RS " << this->rightEncoderSpeed << endl;
}

SensorData * WheelEncoder::getData() {
	EncoderData* data = new EncoderData(this->leftCount, this->rightCount, this->leftCountAbsolute, this->rightCountAbsolute, this->leftEncoderSpeed, this->rightEncoderSpeed);
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
	out << "Left Encoder Speed: " << this->leftEncoderSpeed << endl;
	out << "Right Encoder Speed: " << this->rightEncoderSpeed << endl;
	out << "*******************************" << endl;
}

WheelEncoder::~WheelEncoder() { }
