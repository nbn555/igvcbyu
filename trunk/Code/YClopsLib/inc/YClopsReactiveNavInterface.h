/**
 * @file YclopsReactiveNavInterface.h
 * @date Jan 20, 2011
 * @author igvcbyu
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

#include <mrpt/reactivenav.h>

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

class YClopsReactiveNavInterface: public mrpt::reactivenav::CReactiveInterfaceImplementation{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	YClopsReactiveNavInterface();
	YClopsReactiveNavInterface( mrpt::utils::CConfigFileBase & config );
	virtual ~YClopsReactiveNavInterface();

	bool getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW);
	bool changeSpeeds( float v, float w );
	bool senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles );

	void setAutonomusMode();
	void setNavigationMode();
	void setIdle();

	bool toggleGpsDump();
	bool toggleCompassDump();
	bool toggleLidarDump();
	bool toggleCameraDump();
	bool toggleEncoderDump();

	/**
	 * useYclopsMotorCommand - makes the YclopsReactiveNavInterface use an instance of the
	 * MotorCommand class
	 */
	void useYclopsMotorCommand();

	/**
	 * useYclopsMotorCommand - makes the YclopsReactiveNavInterface use an instance of the
	 * DualMotorCommand class
	 */
	void useWiiMotorCommand();

	/**
	 * useYclopsMotorCommand - makes the YclopsReactiveNavInterface use an instance of the
	 * DummyMotorCommand class
	 */
	void useNullMotorCommand();

private:
	bool isGpsDataShown;
	bool isCompassDataShown;
	bool isLidarDataShown;
	bool isCameraDataShown;
	bool isEncoderDataShown;

	MotorCommandInterface * motor;
	Compass * compass;
	GPS * gps;
	Camera * camera;
	Lidar * lidar;
	WheelEncoder * encoder;

	AbstractPoseEstimator* poseEst;
	mrpt::poses::CPose3D* robotPose;

	float curV;
	float curW;
};

#endif /* YCLOPSREACTIVENAVINTERFACE_H_ */
