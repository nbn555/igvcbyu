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
  motor(NULL), compass(NULL), gps(NULL), camera(NULL), lidar(NULL)
{
	this->motor = new DualMotorCommand();

	if( config.read_bool("COMPASS", "USE", false) ) {
		this->compass = new Compass( config );
	} else {
		LOG(INFO) << "Not using Compass" << endl;
	}

	if( config.read_bool("GPS", "USE", false ) ) {
		this->gps = new GPS();

		gps->setSerialPortName ( config.read_string("GPS", "COM_port_LIN", "/dev/ttyUSB1" ) );
		gps->loadConfig(config, "GPS");
		gps->initConfig(config, "GPS");
		gps->initialize();
	} else {
		LOG(INFO) << "Not using GPS" << endl;
	}

	if( config.read_bool("CAMERA", "USE", false ) ) {
		this->camera = new Camera();
	} else {
		LOG(INFO) << "Not using Camera" << endl;
	}

	if( config.read_bool("LIDAR", "USE", false ) ) {
		//Lidar initialization code
		this->lidar = new CSickLaserSerial();

		this->lidar->setSerialPort( config.read_string("LIDAR", "COM_port_LIN", "/dev/ttyUSB2" ) );
		this->lidar->setBaudRate( config.read_int( "LIDAR", "COM_baudRate", 38400 ) );
		this->lidar->setScanFOV( config.read_int("LIDAR", "FOV", 180 ) );
		this->lidar->setScanResolution( config.read_int( "LIDAR", "resolution", 50 ) );  // 25=0.25deg, 50=0.5deg, 100=1deg
		this->lidar->initialize(); // This will raise an exception on error
	} else {
		LOG(INFO) << "Not using Lidar" << endl;
	}
}

YClopsNavigationSystem2::~YClopsNavigationSystem2() {
	delete this->motor;
	delete this->compass;
	delete this->gps;
	delete this->camera;
	this->lidar->turnOff();
	delete this->lidar;
}

void YClopsNavigationSystem2::doProcess() {
	LOG(DEBUG) << "." << endl;

	if( NULL != this->motor ) {
		this->motor->doProcess();
	}

	if( NULL != this->compass ) {

		this->compass->doProcess();

		if( this->isCompassDataShown ) {
			this->compass->dumpData(cout);
		}
	}

	if( NULL != this->gps ) {

		this->gps->doProcess();

		if( this->isGpsDataShown ) {
			this->gps->dumpData(cout);
		}
	}

	if( NULL != this->camera ) {

		this->camera->doProcess();

		if( this->isCameraDataShown ) {
			this->camera->dumpData(cout);
		}
	}

	if( NULL != this->lidar ) {
		if( this->isLidarDataShown ) {

			static mrpt::gui::CDisplayWindowPlots		win("Laser scans");

			bool						thereIsObservation,hardError;
			CObservation2DRangeScan		obs;

			try
			{
				this->lidar->doProcessSimple( thereIsObservation, obs, hardError );
			}
			catch (std::exception &e)
			{
				cerr << e.what() << endl;
				hardError = true;
			}
			if (hardError)
				LOG(ERROR) << "[TEST] Hardware error=true!!" << endl;

			if (thereIsObservation)
			{
				LOG(DEBUG) << "[TEST] Observation received (" << ((unsigned int)obs.scan.size())
						<< "ranges over " << (RAD2DEG(obs.aperture))
						<< "deg, mid=" << (obs.scan[obs.scan.size()/2])
						<< ")!!"  << endl;
				obs.sensorPose = CPose3D(0,0,0);
				mrpt::slam::CSimplePointsMap		theMap;
				theMap.insertionOptions.minDistBetweenLaserPoints	= 0;
				theMap.insertObservation( &obs );

				vector_float	xs,ys,zs;
				theMap.getAllPoints(xs,ys,zs);
				win.plot(xs,ys,".b3");
				win.axis_equal();
			}
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
