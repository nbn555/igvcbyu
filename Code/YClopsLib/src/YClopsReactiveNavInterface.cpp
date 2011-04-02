/**
 * @file YclopsReactiveNavInterface.cpp
 * @date Jan 20, 2011
 * @author igvcbyu
 * @brief Creates a high level interface for controlling the motor controller and getting obstacle data
 */

#include "YClopsReactiveNavInterface.h"
#include "MotorController.h"
#include "logging.h"
#include "SensorData.h"
#include "YClopsConfiguration.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;

YClopsReactiveNavInterface::YClopsReactiveNavInterface()
: isGpsDataShown(false), isCompassDataShown(false), isLidarDataShown(false), isCameraDataShown(false), isEncoderDataShown(false),
  motor(NULL), compass(NULL), gps(NULL), camera(NULL), lidar(NULL), encoder(NULL), poseEst(NULL), robotPose(NULL),
  curV(0), curW(0)
{
	LOG(DEBUG) << "Creating YClopsReactiveNavInterface" << endl;
	this->motor = new DummyMotorCommand();	//!Initialize a motor controller that is disconnected

	std::string compassName = "COMPASS";	//!Query the config file to see if we are using the compass
	if( YClopsConfiguration::instance().read_bool(compassName, "USE", false) ) {
		LOG(INFO) << "Using Compass" << endl;

		LOG(DEBUG2) << "Creating Compass Object" << endl;
		this->compass = new Compass();		//!create a compass sensor

		LOG(DEBUG2) << "Loading Compass Configuration" << endl;	//!Configure the compass sensor
		this->compass->loadConfig(YClopsConfiguration::instance(), compassName);

		LOG(DEBUG2) << "Initializing Compass" << endl;
		this->compass->initialize();	//!Initialize the compass
	} else {
		LOG(INFO) << "Not using Compass" << endl;
	}

	std::string gpsName = "GPS";
	if( YClopsConfiguration::instance().read_bool(gpsName, "USE", false ) ) {
		LOG(INFO) << "Using GPS" << endl;

		LOG(DEBUG2) << "Creating GPS Object" << endl;
		this->gps = new GPS();

		LOG(DEBUG2) << "Loading GPS Configuration" << endl;
		this->gps->loadConfig(YClopsConfiguration::instance(),gpsName);
		this->gps->loadConfiguration(YClopsConfiguration::instance(),gpsName);

		LOG(DEBUG2) << "Initializing GPS" << endl;
		this->gps->initialize();
	} else {
		LOG(INFO) << "Not using GPS" << endl;
	}

	std::string cameraName = "CAMERA";
	if( YClopsConfiguration::instance().read_bool(cameraName, "USE", false ) ) {
		LOG(INFO) << "Using Camera" << endl;

		LOG(DEBUG2) << "Creating Camera Object" << endl;
		this->camera = new Camera();

		LOG(DEBUG2) << "Loading Camera Configuration" << endl;
		this->camera->loadConfig(YClopsConfiguration::instance(),cameraName);

		LOG(DEBUG2) << "Initializing Camera" << endl;
		this->camera->initialize();
	} else {
		LOG(INFO) << "Not using Camera" << endl;
	}

	std::string lidarName = "LIDAR";
	if( YClopsConfiguration::instance().read_bool(lidarName, "USE", false ) ) {
		LOG(INFO) << "Using Lidar" << endl;

		LOG(DEBUG2) << "Creating Lidar object" << endl;
		this->lidar = new Lidar();

		LOG(DEBUG2) << "Loading Lidar configuration" << endl;
		this->lidar->loadConfiguration(YClopsConfiguration::instance(), lidarName);

		LOG(DEBUG2) << "Initializing Lidar" << endl;
		this->lidar->init();
	} else {
		LOG(INFO) << "Not using Lidar" << endl;
	}

	std::string encoderName = "ENCODER";
	if( YClopsConfiguration::instance().read_bool(encoderName, "USE", false) ) {
		LOG(INFO) << "Using wheel encoders" << endl;

		LOG(DEBUG2) << "Creating WheelEncoder Object" << endl;
		this->encoder = new WheelEncoder();

		LOG(DEBUG2) << "Loading Encoder configuration" << endl;
		this->encoder->loadConfig(YClopsConfiguration::instance(),encoderName);

		LOG(DEBUG2) << "Initializing wheel encoder" << endl;
		this->encoder->initialize();

	} else {
		LOG(INFO) << "Not using wheel encoders" << endl;
	}

	this->robotPose = new CPose3D();				//!create a new robot pose
	this->robotPose->setFromValues(0,0,0,0,0,0);	//!Initialize the robot pose
	LOG(DEBUG2) << "Robot Pose Configured" << endl;	//!@TODO initialize the robot pose with values from the config file

	this->poseEst = new NoFilterPoseEstimator(true);//!Use a no filter pose estimator
	LOG(DEBUG2) << "Using NoFilterPoseEstimator" << endl;	//!@TODO be able to initialize the pose estimator from the config file

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

	CompassData * compassData	= NULL;
	GPSData * gpsData			= NULL;
	EncoderData * encoderData	= NULL;

	//Get get data from the compass
	if( NULL != this->compass ) {
		this->compass->doProcess();

		if( this->isCompassDataShown ) {
			this->compass->dumpData(cout);
		}

		compassData = (CompassData*)(compass->getData());
		LOG_AI(DEBUG4) << "In yclops compass data reactivenav: " << compassData->yaw << " " << compassData->pitch << " " << compassData->roll << endl;
		LOG_AI(DEBUG4) << "Compass data valid? " << compassData->yawValid << " " << compassData->pitchValid << " " << compassData->rollValid << endl;
	}

	//Get data from the gps
	if( NULL != this->gps ) {

		this->gps->sensorProcess();

		if( this->isGpsDataShown ) {
			this->gps->dumpData(cout);
		}

		gpsData = (GPSData*)(gps->getData());
		LOG_AI(DEBUG4) << "GPS data: " << gpsData->latitude << "," << gpsData->longitude << endl;

	}

	//Get data from the encoders
	if( NULL != this->encoder ) {
		LOG(DEBUG4) << "Running Encoder Sensor Process in getCurrentPoseAndSpeed" << endl;
		this->encoder->doProcess();

		if( this->isEncoderDataShown ) {
			this->encoder->dumpData(cout);
		}

		encoderData = (EncoderData*)(encoder->getData());
		LOG_AI(DEBUG4) << "Encoder Data: L " << encoderData->leftCount << " R " << encoderData->rightCount << " Absolute L " <<
				encoderData->leftCountAbsolute << " Absolute R " << encoderData->rightCountAbsolute << endl;
	}

	poseEst->update(gpsData, compassData, encoderData);	// update the pose estimate
	this->poseEst->getPose(*(this->robotPose));			//Get the pose out of the estimate
	curPose = CPose2D(*(this->robotPose));				//return the pose belief state

	this->poseEst->getSpeed(this->curV,this->curW);		//Return the pose speed belief state
	curV = this->curV;
	curW = this->curW;

	LOG_AI(DEBUG4) << "YClopsReactiveNav: Putting into curPose (" << robotPose->x()
			<< "," << robotPose->y() << "," << robotPose->yaw() << ")" << endl;

	//Clean up
	delete compassData;
	delete gpsData;
	delete encoderData;

	return true;
}

bool YClopsReactiveNavInterface::changeSpeeds( float v, float w )
{
	curV = v;
	curW = w;
	bool rval = true;

	if( NULL != this->motor ) {
		motor->setVelocity(v,w);
		motor->doProcess();
		LOG(WARNING) << "Removed the correct return value for change speeds" << endl;
		//rval = motor->getSuccess();
	}
	return rval;
}

bool YClopsReactiveNavInterface::senseObstacles( mrpt::slam::CSimplePointsMap &obstacles )
{
	CameraData * cameraData 	= NULL;
	LidarData * lidarData 		= NULL;

	//Get data out of the Camera
	if( NULL != this->camera ) {

		this->camera->doProcess();

		if( this->isCameraDataShown ) {
			this->camera->dumpData(cout);
		}

		cameraData = (CameraData*)this->camera->getData();
	}

	//Get data out of the lidar
	if( NULL != this->lidar ) {
		this->lidar->sensorProcess();
		if( this->isLidarDataShown ) {
			this->lidar->dumpData(cout);
		}

		lidarData = (LidarData*)this->lidar->getData();
	}

	//!@TODO implement obstacle avoidance

	//clean up
	delete cameraData;
	delete lidarData;

	return true;
}

void YClopsReactiveNavInterface::setAutonomousMode() {
	this->useYclopsMotorCommand();
	LOG(WARNING) << "REMOVED call to useYclopsMotorCommand in setAutonomusMode" << endl;
}

void YClopsReactiveNavInterface::setNavigationMode() {
	this->useYclopsMotorCommand();
	LOG(WARNING) << "REMOVED call to useYclopsMotorCommand in setNavigationMode" << endl;
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
	MotorCommandInterface * tmp = this->motor;
	this->motor = new MotorCommand();
	delete tmp;
}

void YClopsReactiveNavInterface::useWiiMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DualMotorCommand();
	delete tmp;
}

void YClopsReactiveNavInterface::useNullMotorCommand() {
	MotorCommandInterface * tmp = this->motor;
	this->motor = new DummyMotorCommand();
	delete tmp;
}
