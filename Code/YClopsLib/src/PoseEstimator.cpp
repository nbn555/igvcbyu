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
	double lat;
	double lon;
	double x;
	double y;
	double z = 0;

	if( NULL != gpsData ) {
		if(gpsData->valid) {
			lat = gpsData->latitude;
			lon = gpsData->longitude;
		} else {
			LOG_POSE(FATAL) << "No valid data found in gps observation" << endl;
		}
	} else {
		LOG_POSE(FATAL) << "GPS Data pointer NULL" << endl;
	}

	if( NULL != compassData ) {

	}

	if( NULL != encoderData ) {

	}

	if(!started)
	{
		StartLat = lat;
		StartLon = lon;
		this->poseEstimate.setFromValues(lat, lon, z, compassData->yaw, compassData->pitch, compassData->roll);
		started = true;
		return;
	}
	if(convertToMeters){
		double length = AbstractNavigationInterface::haversineDistance(StartLat, StartLon,
				lat, lon);
		double direction = AbstractNavigationInterface::calcBearing(StartLat, StartLon,
				lat, lon);
		x = sin(direction)*length;
		y = cos(direction)*length;

		this->poseEstimate.setFromValues(x, y, z, compassData->yaw, compassData->pitch, compassData->roll);
	} else {
		this->poseEstimate.setFromValues(lat, lon, z, compassData->yaw, compassData->pitch, compassData->roll);
	}

}
