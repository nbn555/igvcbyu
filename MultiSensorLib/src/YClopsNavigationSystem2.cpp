/**
 *  \file: YClopsNavigationSystem2.cpp
 *  \date: Feb 6, 2011
 *  \author: tallred3
 */

#include "YClopsNavigationSystem2.h"
#include <iostream>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;

bool bCameraData;
YClopsNavigationSystem2::YClopsNavigationSystem2(CConfigFile & config)
: isGpsDataShown(false), isCompassDataShown(false), isLidarDataShown(false), isCameraDataShown(false), isEncoderDataShown(false)
{
	this->motor = new DualMotorCommand();
	this->compass = new Compass( config );
	this->gps = new GPS();

	gps->setSerialPortName ( config.read_string("GPS", "COM_port_LIN", "/dev/ttyUSB1" ) );
	gps->loadConfig(config, "GPS");
	gps->initConfig(config, "GPS");
	gps->initialize();

	this->camera = new Camera();
	this->lidar = new CSickLaserSerial();

	this->lidar->setSerialPort("/dev/ttyUSB2");
	this->lidar->setBaudRate(38400);
	this->lidar->setScanFOV(180);
	this->lidar->setScanResolution(50);  // 25=0.25deg, 50=0.5deg, 100=1deg
	cout << "Initializing lidar" << endl;
	this->lidar->initialize(); // This will raise an exception on error
	cout << "Done init lidar" << endl;
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
	cout << "." << endl;

	this->motor->doProcess();
	this->compass->doProcess();
	this->gps->doProcess();
	this->camera->doProcess();

	if( this->isCompassDataShown ) {
		this->compass->dumpData(cout);
	}

	if( this->isGpsDataShown ) {
		this->gps->dumpData(cout);
	}

	if( this->isCameraDataShown ) {
		this->camera->dumpData(cout);
	}

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
			printf("[TEST] Hardware error=true!!\n");
		if (thereIsObservation)
		{
			printf("[TEST] Observation received (%u ranges over %.02fdeg, mid=%.03f)!!\n",
					(unsigned int)obs.scan.size(),
					RAD2DEG(obs.aperture),
					obs.scan[obs.scan.size()/2]);
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
