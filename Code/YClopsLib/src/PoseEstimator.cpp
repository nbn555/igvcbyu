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
	vector_double v(6);
	//this->poseEstimate.getAsVector(v);
	for(int i = 0; i < 6; i++ )
		v[i] = this->poseEstimate[i];
	pose.setFromValues(v[0],v[1],v[2],v[3],v[4],v[5]);
	return true;
}

NoFilterPoseEstimator::NoFilterPoseEstimator(bool inMeters):Meters(inMeters) {
	started = false;

}

NoFilterPoseEstimator::~NoFilterPoseEstimator() {
}
//updates the stored position to the current position
// the first time it will return stuff in lat and lon for the
// TSPSolver after that it returns things in meters from the start point
void NoFilterPoseEstimator::update( const GPSData * gpsData, const CompassData * compassData ) {
	double lat;
	double lon;
	double x;
	double y;
	double z = 0;
	this->yaw = compassData->yaw;

	if(gpsData->valid) {
		lat = gpsData->latitude;
		lon = gpsData->longitude;
	} else {
		LOG_POSE(FATAL) << "No valid data found in gps observation" << endl;
	}

	if(!started)
	{
		StartLat = lat;
		StartLon = lon;
		this->poseEstimate.setFromValues(lat, lon, z, compassData->yaw, compassData->pitch, compassData->roll);
		started = true;
		return;
	}
	if(Meters){
		double length = AbstractNavigationInterface::haversineDistance(StartLat, StartLon,
				lat, lon);
		double direction = AbstractNavigationInterface::calcBearing(StartLat, StartLon,
				lat, lon);
		x = cos(direction)*length;
		y = sin(direction)*length;

		LOG_POSE(DEBUG4) << "Got yaw:" << compassData->yaw << " pitch " << compassData->pitch << " roll " << compassData->roll << endl;
		this->poseEstimate.setFromValues(x, y, z, compassData->yaw, compassData->pitch, compassData->roll);
	} else {
		this->poseEstimate.setFromValues(lat, lon, z, compassData->yaw, compassData->pitch, compassData->roll);
	}

}
