/**
 * @file YclopsReactiveNavInterface.h
 * @date Jan 20, 2011
 * @author igvcbyu
 * @brief YClops robot class, reactive artificial intelligence
 */

#ifndef YCLOPSREACTIVENAVINTERFACE_H_
#define YCLOPSREACTIVENAVINTERFACE_H_

#include "MotorCommandInterface.h"
#include "Compass.h"
#include "GPS.h"
#include "Camera.h"
#include "WheelEncoder.h"
#include "Lidar.h"

#include "PoseEstimator.h"
#include "WaypointPlanner.h"

#include <mrpt/reactivenav.h>

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

class YClopsReactiveNavInterface: public mrpt::reactivenav::CReactiveInterfaceImplementation{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	YClopsReactiveNavInterface();

	/**
	 * @brief Class Destructor
	 */
	virtual ~YClopsReactiveNavInterface();

	/**
	 * @brief returns the present estimate of the pose and speeds
	 * @param[out] curPose The best estimate of the current pose
	 * @param[out] curV The best estimate of the current linear velocity
	 * @param[out] curW The best estimate of the current angular velocity
	 * @return bool true on success
	 */
	bool getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW);

	/**
	 * @brief Sets the robot speeds
	 * @param[in] v The desired linear velocity
	 * @param[in] w The desired angular velocity
	 * @return bool true on success
	 */
	bool changeSpeeds( float v, float w );

	/**
	 * @brief senses obstacles and puts them in the points map
	 * @param[out] obstacles A points map of obstacles
	 * @return bool true on success
	 */
	bool senseObstacles( mrpt::slam::CSimplePointsMap &obstacles );

	/**
	 * @brief Puts the robot into Autonomous Mode
	 */
	void setAutonomousMode();

	/**
	 * @brief Puts the robot into Navigation Mode
	 */
	void setNavigationMode();

	/**
	 * @brief Creates a soft disconnect between the motor controller and the code
	 */
	void setIdle();

	/**
	 * @brief Toggles the output of the gps sensor data to the command line
	 * @return bool true if the gps is dumping data
	 */
	bool toggleGpsDump();

	/**
	 * @brief Toggles the output of the compass sensor data to the command line
	 * @return bool true if the compass is dumping data
	 */
	bool toggleCompassDump();

	/**
	 * @brief Toggles the output of the lidar sensor data to the command line
	 * @return bool true if the lidar is dumping data
	 */
	bool toggleLidarDump();

	/**
	 * @brief Toggles the camera output to the command line
	 * @return bool true if the camera is dumping data
	 */
	bool toggleCameraDump();

	/**
	 * @brief Toggles the Encoder data output to the command line
	 * @return bool true if the encoder is dumping data
	 */
	bool toggleEncoderDump();

	/**
	 * @brief makes the YclopsReactiveNavInterface use an instance of the MotorCommand class
	 */
	void useYclopsMotorCommand();

	/**
	 * @brief makes the YclopsReactiveNavInterface use an instance of the DualMotorCommand class
	 */
	void useWiiMotorCommand();

	/**
	 * @brief makes the YclopsReactiveNavInterface use an instance of the DummyMotorCommand class
	 */
	void useNullMotorCommand();

private:
	bool isGpsDataShown;	//!<if gps is showing data
	bool isCompassDataShown;//!<if compass is showing data
	bool isLidarDataShown;	//!<if lidar is showing data
	bool isCameraDataShown;	//!<if camera is showing data
	bool isEncoderDataShown;//!<if encoder is showing data

	MotorCommandInterface * motor;	//!<Motor interface
	Compass * compass;				//!<Compass Sensor
	GPS * gps;						//!<GPS sensor
	Camera * camera;				//!<Camera sensor
	Lidar * lidar;					//!<Lidar Sensor
	WheelEncoder * encoder;			//!<Wheel Encoder Sensor

	AbstractPoseEstimator* poseEst;	//!<Pose Estimator Interface class
	mrpt::poses::CPose3D* robotPose;//!<The current belief of the robot pose

	float curV;	//!<The current linear velocity
	float curW;	//!<The current angular velocity
};

#endif /* YCLOPSREACTIVENAVINTERFACE_H_ */
