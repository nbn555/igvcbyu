/*
 * YclopsReactiveNavInterface.cpp
 *
 *  Created on: Jan 20, 2011
 *      Author: igvcbyu
 */

#include "YclopsReactiveNavInterface.h"
#include "MotorController.h"
#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

YclopsReactiveNavInterface::YclopsReactiveNavInterface(mrpt::hwdrivers::CConfigFileBase& configSource, string& motorControllerPort) {
	// TODO Auto-generated constructor stub

		gps.setSerialPortName ( "ttyS0" );
		CConfigFile config("GPS.ini");
		gps.loadConfig(config, "GPS");
		gps.initConfig(config, "GPS");
	compass = *(new Compass());

	camera = *(new CCameraSensor());
	camera.loadConfig("camera.ini");
	camera.initialize();
	compass.loadConfig(configSource);
	compass.initialize();
	poseEst = new AbstractPoseEstimator();

	motor = *(new MotorCommand(new MotorController(motorControllerPort)));


}

YclopsReactiveNavInterface::~YclopsReactiveNavInterface() {
	delete gps; gps = 0;
	delete compass; compass = 0;
	delete poseEst; poseEst = 0;
	delete camera; camera = 0;
}
bool YclopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	gps.doProcess();
	compass.doProcess();
	gps.getObservations(lstObs);
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData= CObservationGPSPtr(lstObs.begin()->second);

	poseEst.update(gpsData,compass.getYaw(), compass.getPitch(), compass.getRoll());
	mrpt::poses::CPose3D thirdDim = mrpt::poses::CPose3D(curPose);
	this->poseEst.getPose(thirdDim);
	this->robotPose = *(new CPose3D(thirdDim));
	this->curV = curV;
	this->curW = curW;
	return true;
}
bool YclopsReactiveNavInterface::changeSpeeds( float v, float w )
	{
	//	robotSim.movementCommand(v,w);
		curV = v;
		curW = w;
		return motor.Go(v,w);
	}
bool YclopsReactiveNavInterface::senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles )
	{
		CGenericSensor::TListObservations lstObs;

		camera.doProcess();
		camera.getObservations(lstObs);

		//insert code to get lidar observation into lstObs

		CGenericSensor::TListObservations::iterator itObs = lstObs.begin();
		CGenericSensor::TListObservations::iterator done = lstObs.end();
		obstacles.clear();
		while( itObs != done ) {
		  obstacles.insertObservation(*itObs, robotPose);
		  ++itObs;
		}

		return true;
	}
