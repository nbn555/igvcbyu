/**
 *  \file: YClopsNavigationSystem2.cpp
 *  \date: Feb 6, 2011
 *  \author: tallred3
 */

#include "YClopsNavigationSystem2.h"
#include "logging.h"
#include <iostream>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;

bool bCameraData;
YClopsNavigationSystem2::YClopsNavigationSystem2(CConfigFile & config)
: isGpsDataShown(false), isCompassDataShown(false), isLidarDataShown(false), isCameraDataShown(false), isEncoderDataShown(false),
  motor(NULL), compass(NULL), gps(NULL), camera(NULL), lidar(NULL), encoder(NULL)
{
	this->motor = new DualMotorCommand();

	std::string compassName = "COMPASS";
	if( config.read_bool(compassName, "USE", false) ) {
		LOG(INFO) << "Using Compass" << endl;

		LOG(DEBUG2) << "Creating Compass Object" << endl;
		this->compass = new Compass();

		LOG(DEBUG2) << "Loading Compass Configuration" << endl;
		this->compass->loadConfiguration(config, compassName );

		LOG(DEBUG2) << "Initializing Compass" << endl;
		this->compass->init();
	} else {
		LOG(INFO) << "Not using Compass" << endl;
	}

	std::string gpsName = "GPS";
	if( config.read_bool(gpsName, "USE", false ) ) {
		LOG(INFO) << "Using GPS" << endl;

		LOG(DEBUG2) << "Creating GPS Object" << endl;
		this->gps = new GPS();

		LOG(DEBUG2) << "Loading GPS Configuration" << endl;
		this->gps->loadConfiguration(config,gpsName);

		LOG(DEBUG2) << "Initializing GPS" << endl;
		this->gps->init();
	} else {
		LOG(INFO) << "Not using GPS" << endl;
	}

	std::string cameraName = "CAMERA";
	if( config.read_bool(cameraName, "USE", false ) ) {
		LOG(INFO) << "Using Camera" << endl;

		LOG(DEBUG2) << "Creating Camera Object" << endl;
		this->camera = new Camera();

		LOG(DEBUG2) << "Loading Camera Configuration" << endl;
		this->camera->loadConfiguration(config,cameraName);

		LOG(DEBUG2) << "Initializing Camera" << endl;
		this->camera->init();
	} else {
		LOG(INFO) << "Not using Camera" << endl;
	}

	std::string lidarName = "LIDAR";
	if( config.read_bool(lidarName, "USE", false ) ) {
		LOG(INFO) << "Using Lidar" << endl;

		LOG(DEBUG2) << "Creating Lidar object" << endl;
		this->lidar = new Lidar();

		LOG(DEBUG2) << "Loading Lidar configuration" << endl;
		this->lidar->loadConfiguration(config, lidarName);

		LOG(DEBUG2) << "Initializing Lidar" << endl;
		this->lidar->init();
	} else {
		LOG(INFO) << "Not using Lidar" << endl;
	}

	std::string encoderName = "ENCODER";
	if( config.read_bool(encoderName, "USE", false) ) {
		LOG(INFO) << "Using wheel encoders" << endl;

		LOG(DEBUG2) << "Creating WheelEncoder Object" << endl;
		this->encoder = new WheelEncoder();

		LOG(DEBUG2) << "Loading Encoder configuration" << endl;
		this->encoder->loadConfiguration(config,encoderName);

		LOG(DEBUG2) << "Initializing wheel encoder" << endl;
		this->encoder->init();
	} else {
		LOG(INFO) << "Not using wheel encoders" << endl;
	}

}

YClopsNavigationSystem2::~YClopsNavigationSystem2() {
	delete this->motor;
	delete this->compass;
	delete this->gps;
	delete this->camera;
	delete this->lidar;
	delete this->encoder;
}

void YClopsNavigationSystem2::doProcess() {
	LOG(DEBUG2) << "." << endl;

	if( NULL != this->motor ) {
		this->motor->doProcess();
	}

	if( NULL != this->compass ) {

		this->compass->sensorProcess();

		if( this->isCompassDataShown ) {
			this->compass->dumpData(cout);
		}
	}

	if( NULL != this->gps ) {

		this->gps->sensorProcess();

		if( this->isGpsDataShown ) {
			this->gps->dumpData(cout);
		}
	}

	if( NULL != this->camera ) {

		this->camera->sensorProcess();

		if( this->isCameraDataShown ) {
			this->camera->dumpData(cout);
		}
	}

	if( NULL != this->lidar ) {
		this->lidar->sensorProcess();
		if( this->isLidarDataShown ) {
			this->lidar->dumpData(cout);
		}
	}

	if( NULL != this->encoder ) {
		this->encoder->sensorProcess();

		if( this->isEncoderDataShown ) {
			this->encoder->dumpData(cout);
		}

	}
}

void YClopsNavigationSystem2::setAutonomusMode() {
	this->useYclopsMotorCommand();
}

void YClopsNavigationSystem2::setNavigationMode() {
	this->useYclopsMotorCommand();
}

void YClopsNavigationSystem2::setIdle() {

}

bool YClopsNavigationSystem2::toggleGpsDump() {
	this->isGpsDataShown = !this->isGpsDataShown;
	return this->isGpsDataShown;
}

bool YClopsNavigationSystem2::toggleCompassDump() {
	this->isCompassDataShown = !this->isCompassDataShown;
	return this->isCompassDataShown;
}

bool YClopsNavigationSystem2::toggleLidarDump() {
	this->isLidarDataShown = !this->isLidarDataShown;
	return this->isLidarDataShown;
}

bool YClopsNavigationSystem2::toggleCameraDump() {
	this->isCameraDataShown = !this->isCameraDataShown;
	return this->isCameraDataShown;
}

bool YClopsNavigationSystem2::toggleEncoderDump() {
	this->isEncoderDataShown = !this->isEncoderDataShown;
	return this->isEncoderDataShown;
}

void YClopsNavigationSystem2::useYclopsMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new MotorCommand();
	delete tmp;
}

void YClopsNavigationSystem2::useWiiMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DualMotorCommand();
	delete tmp;
}

void YClopsNavigationSystem2::useNullMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DummyMotorCommand();
	delete tmp;
}
