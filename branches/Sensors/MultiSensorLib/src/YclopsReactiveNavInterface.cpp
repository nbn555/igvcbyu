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

YclopsReactiveNavInterface::YclopsReactiveNavInterface() {

	CConfigFile config("GPS.ini");
	cout << config.getAssociatedFile() << endl;
	gps = new GPS();
	gps->loadConfig(config, "GPS");
	gps->initConfig(config, "GPS");
	gps->initialize();
	cout << "GPS  Configured" << endl;

	camera = NULL;
	//camera = new Camera();
	//CConfigFile camconfig("camera.ini");
	//TODO: Change this to work with ConfigFile Object
//	camera->loadConfig("test");
//	camera->startCamera();

	compass = new Compass(string("Compass.ini"));
	cout << "Compass configured" << endl;

	poseEst = new NoFilterPoseEstimator();

	motor = new MotorCommand();
	cout << "Motor Controller Configured" << endl;
	robotPose = new CPose3D();
	robotPose->setFromValues(0,0,0,0,0,0);
	cout << "Robot Pose Configured" << endl;
	this->curV = 0;
	this->curW = 0;

	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

	bool done = false;
	while(!done)
	{
		while(lstObs.size() == 0)
		{
			compass->doProcess();
			gps->doProcess();
			gps->getObservations(lstObs);
			cout << "Getting Observation from GPS ..." << endl;
		}
		itObs = lstObs.begin();
		CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);
		if(gpsData.pointer()->has_GGA_datum || gpsData.pointer()->has_RMC_datum)
		{
			if(gpsData.pointer()->has_GGA_datum)
				cout << "Got initial point at " << gpsData.pointer()->GGA_datum.latitude_degrees << ":" << gpsData.pointer()->GGA_datum.longitude_degrees << endl;
			if(gpsData.pointer()->has_RMC_datum)
				cout << "Got initial point at " << gpsData.pointer()->RMC_datum.latitude_degrees << ":" << gpsData.pointer()->RMC_datum.longitude_degrees << endl;
			done = true;
		}
		else
		{
			cout << "ERROR: INVALID DATA" << endl;
			gpsData.pointer()->dumpToConsole();
		}

	}
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);

	cout << "In yclops reactivenav" << compass->getYaw() << " " << compass->getPitch() << " " << compass->getRoll() << endl;
	cout << " valid? " << compass->isYawValid() << " " << compass->isPitchValid() << " " << compass->isRollValid() << endl;
	 
	poseEst->update(gpsData,compass->getYaw(), compass->getPitch(), compass->getRoll());

	this->poseEst->getPose(*robotPose);


	cout << "GPS Found observation" << endl;

}

YclopsReactiveNavInterface::~YclopsReactiveNavInterface() {

	delete this->gps;
	delete this->camera;
	delete this->compass;
	delete this->motor;
	delete this->poseEst;
	delete this->robotPose;
}

bool YclopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	gps->doProcess();
	compass->doProcess();
	gps->getObservations(lstObs);
	if(lstObs.size() == 0)
	{
		//return false;

		curPose.x(robotPose->x());
		curPose.y(robotPose->y());
		
		return true;
	}
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);

	cout << "In yclops reactivenav" << compass->getYaw() << " " << compass->getPitch() << " " << compass->getRoll() << endl;
	cout << " valid? " << compass->isYawValid() << " " << compass->isPitchValid() << " " << compass->isRollValid() << endl;

	poseEst->update(gpsData,compass->getYaw(), compass->getPitch(), compass->getRoll());
	mrpt::poses::CPose3D thirdDim = mrpt::poses::CPose3D(curPose);
	this->poseEst->getPose(thirdDim);
	robotPose->x() = thirdDim.x();
	robotPose->y() = thirdDim.y();

	curV = this->curV;
	curW = this->curW;
	curPose.x(robotPose->x());
	curPose.y(robotPose->y());
	curPose.phi(poseEst->getYaw());
	return true;
}

bool YclopsReactiveNavInterface::changeSpeeds( float v, float w )
	{
	//	robotSim.movementCommand(v,w);
		curV = v;
		curW = w;
		motor->setVelocity(v,w);
		motor->doProcess();
		return motor->getSuccess();

	//return false;
	}

bool YclopsReactiveNavInterface::senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles )
	{

	//	camera->getObstacles(obstacles, *robotPose);

		// TODO: Do something with Map here


	/*	CGenericSensor::TListObservations::iterator itObs = camlstObs.begin();
		CGenericSensor::TListObservations::iterator done = camlstObs.end();
		obstacles.clear();
		while( itObs != done ) {
			CObservationPtr observ = CObservationPtr(itObs->second);
			obstacles.insertObservationPtr(observ, &(*robotPose));
			++itObs;
		}
	*/


		return true;
	}


void YclopsReactiveNavInterface::useYclopsMotorCommand() {
	delete this->motor;
	this->motor = new MotorCommand();
}

void YclopsReactiveNavInterface::useWiiMotorCommand() {
	delete this->motor;
	this->motor = new DualMotorCommand();
}

void YclopsReactiveNavInterface::useNullMotorCommand() {
	delete this->motor;
	this->motor = new DummyMotorCommand();
}
