/**
 * @file YclopsReactiveNavInterface.cpp
 * @date Jan 20, 2011
 * @author igvcbyu
 */

#include "YClopsReactiveNavInterface.h"
#include "MotorController.h"
#include "logging.h"
#include "SensorData.h"
//#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;

bool bCameraData;
YClopsReactiveNavInterface::YClopsReactiveNavInterface() {

	CConfigFile config("GPS.ini");
	LOG(INFO) << config.getAssociatedFile() << endl;
	std::string gpsName = "GPS";
	gps = new GPS();
	gps->loadConfiguration(config, gpsName);
	gps->init();
	LOG(DEBUG2) << "GPS  Configured" << endl;

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

YClopsReactiveNavInterface::YClopsReactiveNavInterface(CConfigFileBase & config)
: isGpsDataShown(false), isCompassDataShown(false), isLidarDataShown(false), isCameraDataShown(false), isEncoderDataShown(false),
  motor(NULL), compass(NULL), gps(NULL), camera(NULL), lidar(NULL), encoder(NULL), poseEst(NULL), robotPose(NULL), curV(0), curW(0)
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

	this->robotPose = new CPose3D();
	this->robotPose->setFromValues(0,0,0,0,0,0);
	LOG(DEBUG2) << "Robot Pose Configured" << endl;

	this->poseEst = new NoFilterPoseEstimator();
	LOG(DEBUG2) << "Using NoFilterPoseEstimator" << endl;

	this->curV = 0;
	this->curW = 0;

}


YClopsReactiveNavInterface::~YClopsReactiveNavInterface() {
	LOG(DEBUG4) << "Deleting Motor" << endl;
	delete this->motor;
	LOG(DEBUG4) << "Deleting Compass" << endl;
	delete this->compass;
	LOG(DEBUG4) << "Deleting GPS" << endl;
	delete this->gps;
	LOG(DEBUG4) << "Deleting Camera" << endl;
	delete this->camera;
	LOG(DEBUG4) << "Deleting Lidar" << endl;
	delete this->lidar;
	LOG(DEBUG4) << "Deleting Encoder" << endl;
	delete this->encoder;
	LOG(DEBUG4) << "Deleting poseEst" << endl;
	delete this->poseEst;
	LOG(DEBUG4) << "Deleting robotPose" << endl;
	delete this->robotPose;
}

bool YClopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	LOG(DEBUG2) << "." << endl;

	CompassData * compassData = NULL;
	GPSData * gpsData = NULL;

	//Get get data from the compass
	if( NULL != this->compass ) {
		this->compass->sensorProcess();

		if( this->isCompassDataShown ) {
			this->compass->dumpData(cout);
		}

		compassData = (CompassData*)(compass->getData());
	}

	//Get data from the gps
	if( NULL != this->gps ) {

		this->gps->sensorProcess();

		if( this->isGpsDataShown ) {
			this->gps->dumpData(cout);
		}

		gpsData = (GPSData*)(gps->getData());
	}

	//Get data from the encoders
	if( NULL != this->encoder ) {
		LOG(DEBUG4) << "Running Encoder Sensor Process in getCurrentPoseAndSpeed" << endl;
		this->encoder->sensorProcess();

		if( this->isEncoderDataShown ) {
			this->encoder->dumpData(cout);
		}
		//Incorporate data from the encoders to the position data
	}

	if( NULL == gpsData || NULL == compassData ) {
		curPose.x(robotPose->x());
		curPose.y(robotPose->y());

		delete gpsData;
		delete compassData;
		return true;
	}

	LOG(DEBUG4) << "In yclops compass data reactivenav: " << compassData->yaw << " " << compassData->pitch << " " << compassData->roll << endl;
	LOG(DEBUG4) << "Compass data valid? " << compassData->yawValid << " " << compassData->pitchValid << " " << compassData->rollValid << endl;

	assert(NULL != poseEst);
	assert(NULL != gpsData);
	assert(NULL != compassData);
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

	if(compassData) delete compassData;
	if(gpsData) delete gpsData;

	return true;
}

bool YClopsReactiveNavInterface::changeSpeeds( float v, float w )
{
	curV = v;
	curW = w;
	bool rval = false;

	if( NULL != this->motor ) {
		motor->setVelocity(v,w);
		motor->doProcess();
		rval = motor->getSuccess();
	}
	return rval;
}

bool YClopsReactiveNavInterface::senseObstacles( mrpt::slam::CSimplePointsMap &obstacles )
{

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

	return true;
}

void YClopsReactiveNavInterface::setAutonomusMode() {
	this->useYclopsMotorCommand();
}

void YClopsReactiveNavInterface::setNavigationMode() {
	this->useYclopsMotorCommand();
}

void YClopsReactiveNavInterface::setIdle() {

}

bool YClopsReactiveNavInterface::toggleGpsDump() {
	if( NULL != this->gps )
		this->isGpsDataShown = !this->isGpsDataShown;
	else this->isGpsDataShown = false;
	return this->isGpsDataShown;
}

bool YClopsReactiveNavInterface::toggleCompassDump() {
	if( NULL != this->compass )
		this->isCompassDataShown = !this->isCompassDataShown;
	else this->isCompassDataShown = false;
	return this->isCompassDataShown;
}

bool YClopsReactiveNavInterface::toggleLidarDump() {
	if( NULL != this->lidar )
		this->isLidarDataShown = !this->isLidarDataShown;
	else this->isLidarDataShown = false;
	return this->isLidarDataShown;
}

bool YClopsReactiveNavInterface::toggleCameraDump() {
	if( NULL != this->camera )
		this->isCameraDataShown = !this->isCameraDataShown;
	else this->isCameraDataShown = false;
	return this->isCameraDataShown;
}

bool YClopsReactiveNavInterface::toggleEncoderDump() {
	if( NULL != this->encoder )
		this->isEncoderDataShown = !this->isEncoderDataShown;
	else this->isEncoderDataShown = false;
	return this->isEncoderDataShown;
}

void YClopsReactiveNavInterface::useYclopsMotorCommand() {
	delete this->motor;
	this->motor = new MotorCommand();
}

void YClopsReactiveNavInterface::useWiiMotorCommand() {
	delete this->motor;
	this->motor = new DualMotorCommand();
}

void YClopsReactiveNavInterface::useNullMotorCommand() {
	delete this->motor;
	this->motor = new DummyMotorCommand();
}
