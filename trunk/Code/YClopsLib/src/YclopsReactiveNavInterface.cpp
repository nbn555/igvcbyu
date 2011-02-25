/**
 * @file YclopsReactiveNavInterface.cpp
 * @date Jan 20, 2011
 * @author igvcbyu
 */

#include "YclopsReactiveNavInterface.h"
#include "MotorController.h"
#include "logging.h"
#include "SensorData.h"
//#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

YclopsReactiveNavInterface::YclopsReactiveNavInterface() {

	CConfigFile config("GPS.ini");
	LOG(INFO) << config.getAssociatedFile() << endl;
	std::string gpsName = "GPS";
	gps = new GPS();
	gps->loadConfiguration(config, gpsName);
	gps->init();
	LOG(DEBUG4) << "GPS  Configured" << endl;

	camera = NULL;
	//camera = new Camera();
	//CConfigFile camconfig("camera.ini");
	//TODO: Change this to work with ConfigFile Object
//	camera->loadConfig("test");
//	camera->startCamera();


	std::string compassName = "COMPASS";
	CConfigFile compassConfig("Compass.ini");
	this->compass = new Compass();
	this->compass->loadConfiguration(compassConfig, compassName );
	this->compass->init();
	LOG(DEBUG4) << "Compass configured" << endl;

	poseEst = new NoFilterPoseEstimator();

	motor = new MotorCommand();
	LOG(DEBUG4) << "Motor Controller Configured" << endl;
	robotPose = new CPose3D();
	robotPose->setFromValues(0,0,0,0,0,0);
	LOG(DEBUG4) << "Robot Pose Configured" << endl;
	this->curV = 0;
	this->curW = 0;

	CompassData * compassData = NULL;
	GPSData * gpsData = NULL;
	bool done = false;
	while(!done)
	{
		do {
			compass->sensorProcess();
			gps->sensorProcess();

			compassData = (CompassData*)(compass->getData());
			gpsData = (GPSData*)(gps->getData());
			LOG(DEBUG4) << "Getting Observation from GPS ..." << endl;
		} while(!gpsData->valid);

		if(gpsData->valid) {
			LOG(DEBUG3) << "Got initial point at " << gpsData->latitude << ":" << gpsData->longitude << endl;
			done = true;
		} else {
			LOG(ERROR) << "INVALID DATA" << endl;
		}
	}

	LOG(DEBUG4) << "In yclops reactivenav" << compassData->yaw << " " << compassData->pitch << " " << compassData->roll << endl;
	LOG(DEBUG4) << " valid? " << compassData->yawValid << " " << compassData->pitchValid << " " << compassData->rollValid << endl;
	 
	poseEst->update( gpsData, compassData);

	this->poseEst->getPose(*robotPose);

	LOG(DEBUG4) << "GPS Found observation" << endl;

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
	gps->sensorProcess();
	compass->sensorProcess();

	GPSData * gpsData = (GPSData*)(gps->getData());
	CompassData * compassData = (CompassData*)(compass->getData());

	if(!gpsData->valid)
	{
		//return false;

		curPose.x(robotPose->x());
		curPose.y(robotPose->y());
		
		delete gpsData;
		delete compassData;

		return true;
	}

	LOG(DEBUG4) << "In yclops reactivenav" << compassData->yaw << " " << compassData->pitch << " " << compassData->roll << endl;
	LOG(DEBUG4) << " valid? " << compassData->yawValid << " " << compassData->pitchValid << " " << compassData->rollValid << endl;

	poseEst->update(gpsData,compassData);
	mrpt::poses::CPose3D thirdDim = mrpt::poses::CPose3D(curPose);
	this->poseEst->getPose(thirdDim);
	robotPose->x() = thirdDim.x();
	robotPose->y() = thirdDim.y();

	curV = this->curV;
	curW = this->curW;
	curPose.x(robotPose->x());
	curPose.y(robotPose->y());
	curPose.phi(poseEst->getYaw());

	delete gpsData;
	delete compassData;

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
