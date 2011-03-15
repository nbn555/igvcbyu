/**
 * @file YclopsReactiveNavInterface.cpp
 * @date Jan 20, 2011
 * @author igvcbyu
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
  motor(NULL), compass(NULL), gps(NULL), camera(NULL), lidar(NULL), encoder(NULL), poseEst(NULL), robotPose(NULL), nav(NULL),
  curV(0), curW(0)
{
	this->motor = new DummyMotorCommand();

	std::string compassName = "COMPASS";
	if( YClopsConfiguration::instance().read_bool(compassName, "USE", false) ) {
		LOG(INFO) << "Using Compass" << endl;

		LOG(DEBUG2) << "Creating Compass Object" << endl;
		this->compass = new Compass();

		LOG(DEBUG2) << "Loading Compass Configuration" << endl;
		this->compass->loadConfiguration(YClopsConfiguration::instance(), compassName );

		LOG(DEBUG2) << "Initializing Compass" << endl;
		this->compass->init();
	} else {
		LOG(INFO) << "Not using Compass" << endl;
	}

	std::string gpsName = "GPS";
	if( YClopsConfiguration::instance().read_bool(gpsName, "USE", false ) ) {
		LOG(INFO) << "Using GPS" << endl;

		LOG(DEBUG2) << "Creating GPS Object" << endl;
		this->gps = new GPS();

		LOG(DEBUG2) << "Loading GPS Configuration" << endl;
		this->gps->loadConfiguration(YClopsConfiguration::instance(),gpsName);

		LOG(DEBUG2) << "Initializing GPS" << endl;
		this->gps->init();
	} else {
		LOG(INFO) << "Not using GPS" << endl;
	}

	std::string cameraName = "CAMERA";
	if( YClopsConfiguration::instance().read_bool(cameraName, "USE", false ) ) {
		LOG(INFO) << "Using Camera" << endl;

		LOG(DEBUG2) << "Creating Camera Object" << endl;
		this->camera = new Camera();

		LOG(DEBUG2) << "Loading Camera Configuration" << endl;
		this->camera->loadConfiguration(YClopsConfiguration::instance(),cameraName);

		LOG(DEBUG2) << "Initializing Camera" << endl;
		this->camera->init();
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
		this->encoder->loadConfiguration(YClopsConfiguration::instance(),encoderName);

		LOG(DEBUG2) << "Initializing wheel encoder" << endl;
		this->encoder->init();

	} else {
		LOG(INFO) << "Not using wheel encoders" << endl;
	}

	this->robotPose = new CPose3D();
	this->robotPose->setFromValues(0,0,0,0,0,0);
	LOG(DEBUG2) << "Robot Pose Configured" << endl;

	this->poseEst = new NoFilterPoseEstimator(true);
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
	LOG(DEBUG4) << "Deleting nav interface" << endl;
	delete this->nav;
}

bool YClopsReactiveNavInterface::getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	LOG(DEBUG2) << "." << endl;

	CompassData * compassData = NULL;
	GPSData * gpsData = NULL;
	EncoderData * encoderData = NULL;

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

		encoderData = (EncoderData*)(encoder->getData());
	}

	if( NULL == gpsData || NULL == compassData /*|| NULL == encoderData*/ ) {
		curPose.x(robotPose->x());
		curPose.y(robotPose->y());

		delete gpsData;
		delete compassData;
		//delete encoderData;
		return true;
	}

	assert(NULL != poseEst);
	assert(NULL != gpsData);
	assert(NULL != compassData);
	//assert(NULL != encoderData);

	LOG_AI(DEBUG4) << "GPS data: " << gpsData->latitude << "," << gpsData->longitude << endl;
	LOG_AI(DEBUG4) << "In yclops compass data reactivenav: " << compassData->yaw << " " << compassData->pitch << " " << compassData->roll << endl;
	LOG_AI(DEBUG4) << "Compass data valid? " << compassData->yawValid << " " << compassData->pitchValid << " " << compassData->rollValid << endl;
	//LOG_AI(DEBUG4) << "Encoder Data: L " << encoderData->leftCount << " R " << encoderData->rightCount << " Absolute L " <<
	//		encoderData->leftCountAbsolute << " Absolute R " << encoderData->rightCountAbsolute << endl;

	poseEst->update(gpsData, compassData, encoderData);
	this->poseEst->getPose(*(this->robotPose));
	curPose = CPose2D(*(this->robotPose));

	this->poseEst->getSpeed(this->curV,this->curW);
	curV = this->curV;
	curW = this->curW;

	LOG_AI(DEBUG4) << "YClopsReactiveNav: Putting into curPose (" << robotPose->x() << "," << robotPose->y() << "," << poseEst->getYaw() << ")" << endl;

	if(compassData) delete compassData;
	if(gpsData) delete gpsData;
	if(encoderData) delete encoderData;

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
		LOG(WARNING) << "Removed the correct return falue for change speeds" << endl;
		//rval = motor->getSuccess();
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
//	this->useYclopsMotorCommand();
	LOG(WARNING) << "REMOVED call to useYclopsMotorCommand in setAutonomusMode" << endl;
	if(NULL != this->nav ) delete this->nav;
	this->nav = new SequentialNavigation(this->robotPose->x(),this->robotPose->y());
}

void YClopsReactiveNavInterface::setNavigationMode() {
//	this->useYclopsMotorCommand();
	LOG(WARNING) << "REMOVED call to useYclopsMotorCommand in setNavigationMode" << endl;
	if(NULL != this->nav ) delete this->nav;
	this->nav = new TSPNavigation(this->robotPose->x(), this->robotPose->y());
}

void YClopsReactiveNavInterface::setIdle() {
	if( NULL != this->nav ) delete this->nav;
	this->nav = NULL;
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
