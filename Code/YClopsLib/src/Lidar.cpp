/*
 * Lidar.cpp
 *
 *  Created on: Feb 25, 2011
 *      Author: tallred3
 */

#include "Lidar.h"
#include "logging.h"
#include <string>
#include <mrpt/gui.h>
#include <mrpt/slam.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

Lidar::Lidar() {

}

void Lidar::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	string portName = config.read_string(sectionName, "COM_port_LIN", "/dev/ttyUSB2" );
	LOG(DEBUG3) << sectionName << " port name: " << portName << endl;
	this->setSerialPort( portName );

	int baudRate = config.read_int( sectionName, "COM_baudRate", 38400 );
	LOG(DEBUG3) << sectionName << " baudRate: " << baudRate << endl;
	this->setBaudRate( baudRate );

	int fov = config.read_int( sectionName, "FOV", 180 );
	LOG(DEBUG3) << sectionName << " fov: " << fov << endl;
	this->setScanFOV( fov );

	int res = config.read_int( sectionName, "resolution", 50 );
	LOG(DEBUG3) << sectionName << "resolution: " << res / 100. << endl;
	this->setScanResolution( config.read_int( sectionName, "resolution", 50 ) );  // 25=0.25deg, 50=0.5deg, 100=1deg

}

void Lidar::init() {
	this->initialize(); // This will raise an exception on error
}

void Lidar::sensorProcess() {

}

SensorData * Lidar::getData() {
	return new LidarData();
}

void Lidar::dumpData( std::ostream & out ) {
	static mrpt::gui::CDisplayWindowPlots		win("Laser scans");

	bool						thereIsObservation, hardError;
	CObservation2DRangeScan		obs;

	try
	{
		this->doProcessSimple( thereIsObservation, obs, hardError );
	}
	catch (std::exception &e)
	{
		LOG(ERROR) << "[TEST] Hardware error=true!!" << endl;
		LOG(ERROR) << e.what() << endl;
	}

	if (thereIsObservation)
	{
		LOG(DEBUG3) << "[TEST] Observation received (" << ((unsigned int)obs.scan.size())
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

Lidar::~Lidar() {
	LOG(DEBUG3) << "Turning off Lidar" << endl;
	this->turnOff();
}
