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
	//	gps->setSerialPortName ( "ttyUSB0" );
		CConfigFile config("GPS.ini");
		gps.loadConfig(config, "GPS");
		gps.initConfig(config, "GPS");
		gps.initialize();
		cout << "GPS  Configured" << endl;



	//camera = new Camera();
	//CConfigFile camconfig("camera.ini");
	//TODO: Change this to work with ConfigFile Object
//	camera->loadConfig("test");
//	camera->startCamera();



	compass = new Compass();
	CConfigFile config2( "Compass.ini" );

	compass->loadConfig(config2, "COMPASS" );
	compass->initialize();

	cout << "Compass configered" << endl;

	poseEst = new NoFilterPoseEstimator();

	motor = new MotorCommand(new MotorController(motorControllerPort));

	robotPose = new CPose3D();
	robotPose->setFromValues(0,0,0,0,0,0);

	this->curV = 0;
	this->curW = 0;

	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

	bool done = false;
	while(!done)
	{
		while(lstObs.size() == 0)
		{
			gps.doProcess();
			gps.getObservations(lstObs);
		}
		itObs = lstObs.begin();
		CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);
		if(gpsData.pointer()->GGA_datum.latitude_degrees)
		{
			done = true;
		}
		else
		{
			cout << "Got Bad Data" << endl;
		}

	}
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);

	poseEst->update(gpsData,compass->getYaw(), compass->getPitch(), compass->getRoll());

	this->poseEst->getPose(*robotPose);


	cout << "GPS Found observation" << endl;

}

YclopsReactiveNavInterface::~YclopsReactiveNavInterface() {

}

bool YclopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CGenericSensor::TListObservations	lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;
	gps.doProcess();
	(*compass).doProcess();
	gps.getObservations(lstObs);
	if(lstObs.size() == 0)
	{
		//return false;

		curPose.x(robotPose->x());
		curPose.y(robotPose->y());
		
		return true;
	}
	itObs = lstObs.begin();
	CObservationGPSPtr gpsData = CObservationGPSPtr(lstObs.begin()->second);

	poseEst->update(gpsData,compass->getYaw(), compass->getPitch(), compass->getRoll());
	mrpt::poses::CPose3D thirdDim = mrpt::poses::CPose3D(curPose);
	this->poseEst->getPose(thirdDim);
	robotPose->x() = thirdDim.x();
	robotPose->y() = thirdDim.y();

	curV = this->curV;
	curW = this->curW;
	curPose.x(robotPose->x());
	curPose.y(robotPose->y());
	return true;
}

bool YclopsReactiveNavInterface::changeSpeeds( float v, float w )
	{
	//	robotSim.movementCommand(v,w);
		curV = v;
		curW = w;
		return motor->Go(v,w);

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
double YclopsReactiveNavInterface::getHeading()
{
	return robotPose->yaw();
}

