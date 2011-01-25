/*
 * AbstractPoseEstimator.cpp
 *
 *  Created on: Dec 27, 2010
 *      Author: tallred3
 */

#include "PoseEstimator.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;

AbstractPoseEstimator::AbstractPoseEstimator() {
	// TODO Auto-generated constructor stub

}

AbstractPoseEstimator::~AbstractPoseEstimator() {
	// TODO Auto-generated destructor stub
}

bool AbstractPoseEstimator::getPose( CPose3D & pose ) {
	vector_double v(6);
	this->poseEstimate.getAsVector(v);
	pose.setFromValues(v[0],v[1],v[2],v[3],v[4],v[5]);
	return true;
}

NoFilterPoseEstimator::NoFilterPoseEstimator() {
	// TODO Auto-generated constructor stub

}

NoFilterPoseEstimator::~NoFilterPoseEstimator() {
	// TODO Auto-generated destructor stub
}

void NoFilterPoseEstimator::update( mrpt::slam::CObservationGPSPtr gpsObsPtr, double yaw, double pitch, double roll ) {
	double lat;
	double lon;
	double x;
	double y;
	double z = 0;

	if( gpsObsPtr->has_GGA_datum ){
		lat = gpsObsPtr->GGA_datum.latitude_degrees;
		lon = gpsObsPtr->GGA_datum.longitude_degrees;
		z = gpsObsPtr->GGA_datum.altitude_meters;
	} else if( gpsObsPtr->has_RMC_datum ) {
		lat = gpsObsPtr->RMC_datum.latitude_degrees;
		lon = gpsObsPtr->RMC_datum.longitude_degrees;
	} else {
		cerr << "No valid data found in gps observation" << endl;
		exit(EXIT_FAILURE);
	}

	x = lon;
	y = lat;

	this->poseEstimate.setFromValues(x, y, z, yaw, pitch, roll);

}
