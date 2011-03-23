/*
 * AbstractPoseEstimator.cpp
 *
 *  Created on: Dec 27, 2010
 *      Author: tallred3
 */

#include "PoseEstimator.h"
#include "WaypointPlanner.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;

AbstractPoseEstimator::AbstractPoseEstimator() {
}

AbstractPoseEstimator::~AbstractPoseEstimator() {
}

bool AbstractPoseEstimator::getPose( CPose3D & pose ) {
	pose = this->poseEstimate;
	return true;
}

bool AbstractPoseEstimator::getSpeed( float & curv, float & curw ){
	curv = this->curV;
	curw = this->curW;
	return false;
}

NoFilterPoseEstimator::NoFilterPoseEstimator(bool convertToMeters):started(false), convertToMeters(convertToMeters) {
}

NoFilterPoseEstimator::~NoFilterPoseEstimator() {
}
//updates the stored position to the current position
// the first time it will return stuff in lat and lon for the
// TSPSolver after that it returns things in meters from the start point
void NoFilterPoseEstimator::update( const GPSData * gpsData, const CompassData * compassData, const EncoderData * encoderData ) {
	double lat = this->poseEstimate.x();
	double lon = this->poseEstimate.y();
	double x;
	double y;
	double z 		= 0;
	double yaw		= 0;
	double pitch	= 0;
	double roll		= 0;
	bool gpsValid = false;

	if( NULL != gpsData ) {
		if(gpsData->valid) {
			lat = gpsData->latitude;
			lon = gpsData->longitude;
			gpsValid = true;
		} else {
			LOG_POSE(ERROR) << "No valid data found in gps observation" << endl;
		}
	} else {
		LOG_POSE(ERROR) << "GPS Data pointer NULL" << endl;
	}

	if( NULL != compassData ) {
		if(compassData->yawValid) {
			yaw = compassData->yaw;
		} else {
			LOG_POSE(ERROR) << "No valid yaw data in compass observation" << endl;
		}
		if(compassData->pitchValid) {
			pitch = compassData->pitch;
		} else {
			LOG_POSE(ERROR) << "No valid pitch data in compass observation" << endl;
		}
		if(compassData->rollValid) {
			roll = compassData->roll;
		} else {
			LOG_POSE(ERROR) << "No valid roll data in compass observation" << endl;
		}
	} else {
		LOG_POSE(ERROR) << "Compass Data pointer NULL" << endl;
	}

	if( NULL != encoderData ) {
		//!@TODO Implement encoderData in pose estimators
	} else {
		LOG_POSE(ERROR) << "Encoder Data pointer NULL" << endl;
	}

	if(!started)
	{
		StartLat = lat;
		StartLon = lon;
		this->poseEstimate.setFromValues(lat, lon, z, yaw, pitch, roll);
		started = true;
		return;
	}
	if(convertToMeters && gpsValid){
		double length = AbstractNavigationInterface::haversineDistance(StartLat, StartLon,
				lat, lon);
		double direction = AbstractNavigationInterface::calcBearing(StartLat, StartLon,
				lat, lon);
		x = sin(direction)*length;
		y = -cos(direction)*length;

		this->poseEstimate.setFromValues(x, y, z, yaw, pitch, roll);
	} else {
		this->poseEstimate.setFromValues(lat, lon, z, yaw, pitch, roll);
	}

}
