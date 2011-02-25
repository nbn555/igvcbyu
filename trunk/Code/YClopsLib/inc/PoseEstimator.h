/**
 * @file AbstractPoseEstimator.h
 * @date Dec 27, 2010
 * @author T. Eldon Allred
 */

#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/poses/CPose3D.h>
#include "SensorData.h"
#include "logging.h"

class AbstractPoseEstimator {
public:
	AbstractPoseEstimator();
	virtual ~AbstractPoseEstimator();

	/**
	 * updates the estimate of the position
	 * @param yaw - the rotation about the y axis measured in radians
	 * @param pitch - the rotation about the x axis measured in radians
	 * @param roll - the rotation about the z axis measured in radians
	 */
	virtual void update( const GPSData * gpsObsPtr, const CompassData * compassData ) = 0;
	double getYaw(void){ return yaw;}
	bool getPose( mrpt::poses::CPose3D & pose );
protected:
	mrpt::poses::CPose3D poseEstimate;
	double yaw;
};

class NoFilterPoseEstimator: public AbstractPoseEstimator {
public:
	bool started;
	bool Meters;
	double StartLat;
	double StartLon;
	NoFilterPoseEstimator(bool inMeters = false);
	virtual ~NoFilterPoseEstimator();

	/**
	 * NoFilterPoseEstimator::update - dummy implementation does no filtering just takes the
	 * direct value from the sensor
	 */
	void update( const GPSData * gpsObsPtr, const CompassData * compassData );
};

#endif /* POSEESTIMATOR_H_ */
