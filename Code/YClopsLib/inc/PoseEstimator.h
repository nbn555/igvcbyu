/**
 * @file AbstractPoseEstimator.h
 * @date Dec 27, 2010
 * @author T. Eldon Allred
 * @brief Estimator classes for the robot pose
 */

#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <mrpt/poses/CPose3D.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include "SensorData.h"
#include "logging.h"

/**
 * @brief Pose Estimator parent class interface
 */
class AbstractPoseEstimator {
public:
	AbstractPoseEstimator();
	virtual ~AbstractPoseEstimator();

	/**
	 * @brief updates the estimate of the position
	 * @param[in] gpsData the gps data observation
	 * @param[in] compassData the compass data observation
	 * @param[in] encoderData the encoder data observation
	 * @todo change this to be a vector of SensorData to abstract the kind of data we are passing (it will be more dynamic, but require rtti)
	 */
	virtual void update( const GPSData * gpsData, const CompassData * compassData, const EncoderData * encoderData ) = 0;

	/**
	 * @brief gets the current belief of the pose
	 * @param[out] pose the pose belief
	 * @return bool true if successful
	 */
	bool getPose( mrpt::poses::CPose3D & pose );

	/**
	 * @brief gets the current belief of the speed
	 * @param[out] curV the current linear velocity
	 * @param[out] curW the current angular velocity
	 * @return bool true on success
	 */
	bool getSpeed( float & curV, float & curW );
protected:
	mrpt::poses::CPose3D poseEstimate;	//!<The current pose belief of the robot
	double curV;						//!<The current linear velocity belief of the robot
	double curW;						//!<The current angular velocity belief of the robot
};

/**
 * @brief a no filter class just copies the observation to the output
 */
class NoFilterPoseEstimator: public AbstractPoseEstimator {
public:
	bool started;
	bool convertToMeters;
	double StartLat;
	double StartLon;

	NoFilterPoseEstimator(bool convertToMeters = false);
	virtual ~NoFilterPoseEstimator();

	/**
	 * @brief dummy implementation does no filtering just takes the direct value from the sensor
	 */
	void update( const GPSData * gpsData, const CompassData * compassData, const EncoderData * encoderData );
};

/**
 * @brief A kalman filtering class mrpt supports ekf ikf and another one
 * @todo implement this class
 */
class RangeBearingPoseEstimator: public AbstractPoseEstimator {
protected:
	class RangeBearing: public mrpt::bayes::CKalmanFilterCapable<4,2,2,1>{




	};

public:

	/**
	 * @brief KalmanFilter update
	 * @todo implement this function
	 */
	void update( const GPSData * gpsData, const CompassData * compassData, const EncoderData * encoderData );

};

#endif /* POSEESTIMATOR_H_ */
