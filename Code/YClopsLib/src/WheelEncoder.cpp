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

WheelEncoder::WheelEncoder(): left(0, 0), right(0, 0), encoderPPR(0) { }

void WheelEncoder::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	double leftDiam = config.read_double( sectionName, "WHEEL_DIAMETER_LEFT", 0.30 );
	double rightDiam = config.read_double( sectionName, "WHEEL_DIAMETER_RIGHT", 0.30 );
	this->encoderPPR = config.read_int( sectionName, "ENCODER_PPR", 440 );

	this->left = encoder( encoderPPR, leftDiam );
	this->right = encoder( encoderPPR, rightDiam );

}

void WheelEncoder::init() {

	LOG_ENCODER(DEBUG4) << "Setting encoder usage to feedback" << endl;
	MotorController::instance()->setEncoderUsage();

	LOG_ENCODER(DEBUG4) << "Setting the EncoderPPR" << endl;
	MotorController::instance()->setEncoderPPR(this->encoderPPR);

	LOG_ENCODER(DEBUG4) << "Setting encoder settings" << endl;
	MotorController::instance()->setEncoderCounter(MotorController::BothChannels,0);
}

void WheelEncoder::sensorProcess() {
	int tmp1 = 0, tmp2 = 0;
	LOG_ENCODER(DEBUG4) << "Getting Absolute Encoder Values" << endl;
	MotorController::instance()->getAbsoluteEncoderCount(this->left.absoluteCount, this->right.absoluteCount);
	LOG_ENCODER(DEBUG4) << "Getting Relative Encoder Values" << endl;
	MotorController::instance()->getRelativeEncoderCount(tmp1,tmp2);
	this->left.count += tmp1;
	this->right.count += tmp2;
	LOG_ENCODER(DEBUG4) << "Getting Encoder Speed" << endl;
	MotorController::instance()->getEncoderSpeeds(this->left.speed, this->right.speed);

	LOG_ENCODER(DEBUG3) << "Left Encoder value: " << this->left.count << endl;
	LOG_ENCODER(DEBUG3) << "Right Encoder value: " << this->right.count << endl;
	LOG_ENCODER(DEBUG3) << "Left Absolute value: " << this->left.absoluteCount << endl;
	LOG_ENCODER(DEBUG3) << "Right Absolute value: " << this->right.absoluteCount << endl;
	LOG_ENCODER(DEBUG3) << "Left Encoder Speed: " << this->left.speed << endl;
	LOG_ENCODER(DEBUG3) << "Right Encoder Speed: " << this->right.speed << endl;
	LOG_ENCODER(DEBUG3) << "Left Distance: " << this->left.getDistance() << endl;
	LOG_ENCODER(DEBUG3) << "Right Distance: " << this->right.getDistance() << endl;
	LOG_ENCODER(DEBUG3) << "Left Distance Absolute: " << this->left.getTotalDistance() << endl;
	LOG_ENCODER(DEBUG3) << "Right Distance Absolute: " << this->right.getTotalDistance() << endl;
	LOG_ENCODER(DEBUG3) << "Left speed: " << this->left.getSpeed() << endl;
	LOG_ENCODER(DEBUG3) << "Right speed: " << this->right.getSpeed() << endl;


}

SensorData * WheelEncoder::getData() {
	EncoderData* data = new EncoderData(this->left.count, this->right.count, this->left.absoluteCount, this->right.absoluteCount, this->left.speed, this->right.speed, this->left.getDistance(), this->right.getDistance() );
	this->left.count = 0;
	this->right.count = 0;
	return data;
}

void WheelEncoder::dumpData( std::ostream & out ) const {
	out << "*******************************" << endl;
	out << "Left Encoder value: " << this->left.count << endl;
	out << "Right Encoder value: " << this->right.count << endl;
	out << "Left Absolute value: " << this->left.absoluteCount << endl;
	out << "Right Absolute value: " << this->right.absoluteCount << endl;
	out << "Left Encoder Speed: " << this->left.speed << endl;
	out << "Right Encoder Speed: " << this->right.speed << endl;
	out << "Left Distance: " << this->left.getDistance() << endl;
	out << "Right Distance: " << this->right.getDistance() << endl;
	out << "Left Distance Absolute: " << this->left.getTotalDistance() << endl;
	out << "Right Distance Absolute: " << this->right.getTotalDistance() << endl;
	out << "Left speed: " << this->left.getSpeed() << endl;
	out << "Right speed: " << this->right.getSpeed() << endl;
	out << "*******************************" << endl;
}

WheelEncoder::~WheelEncoder() { }
