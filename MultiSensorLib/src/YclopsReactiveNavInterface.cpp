/**
 * @file YclopsReactiveNavInterface.cpp
 * @date Jan 20, 2011
 * @author igvcbyu
 */

#include "YclopsReactiveNavInterface.h"
#include "MotorController.h"
//#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

YclopsReactiveNavInterface::YclopsReactiveNavInterface(string& motorControllerPort) {
	// TODO Auto-generated constructor stub
		gps = new GPS();
	//	gps->setSerialPortName ( "ttyUSB0" );
		CConfigFile config("GPS.ini");
		gps->loadConfig(config, "GPS");
		gps->initConfig(config, "GPS");
		gps->initialize();


	camera = new CCameraSensor();
	CConfigFile camconfig("camera.ini");
	camera->loadConfig(camconfig,"CAMERA");
	camera->initialize();

	compass = new Compass();
	CConfigFile config2( "Compass.ini" );

	compass->loadConfig(config2, "COMPASS" );
	compass->initialize();
	poseEst = new NoFilterPoseEstimator();

	motor = new MotorCommand(new MotorController(motorControllerPort));

	robotPose = new CPose3D();
	robotPose->setFromValues(0,0,0,0,0,0);

	this->curV = 0;
	this->curW = 0;
}

YclopsReactiveNavInterface::~YclopsReactiveNavInterface() {

}

bool YclopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	gps->doProcess();
	compass->doProcess();
	gps->getObservations(lstObs);
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);

	poseEst->update(gpsData,compass->getYaw(), compass->getPitch(), compass->getRoll());
	mrpt::poses::CPose3D thirdDim = mrpt::poses::CPose3D(curPose);
	this->poseEst->getPose(thirdDim);
	this->robotPose = new CPose3D(thirdDim);
	this->curV = curV;
	this->curW = curW;
	return true;
}

bool YclopsReactiveNavInterface::changeSpeeds( float v, float w )
	{
	//	robotSim.movementCommand(v,w);
		curV = v;
		curW = w;
		return motor->Go(v,w);

	return false;
	}

bool YclopsReactiveNavInterface::senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles )
	{
		CGenericSensor::TListObservations lstObs;

		camera->doProcess();
		camera->getObservations(lstObs);

		//insert code to get lidar observation into lstObs

		CGenericSensor::TListObservations::iterator itObs = lstObs.begin();
		CGenericSensor::TListObservations::iterator done = lstObs.end();
		obstacles.clear();
		while( itObs != done ) {
			CObservationPtr observ = CObservationPtr(itObs->second);
			obstacles.insertObservationPtr(observ, &(*robotPose));

			++itObs;
		}

		return true;
	}
