/*
 * YclopsReactiveNavInterface.h
 *
 *  Created on: Jan 20, 2011
 *      Author: igvcbyu
 */

#ifndef YCLOPSREACTIVENAVINTERFACE_H_
#define YCLOPSREACTIVENAVINTERFACE_H_
#include "Compass.h"
#include "GPS.h"
#include "MotorCommand.h"
#include "PoseEstimator.h"
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/reactivenav.h>

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::hwdrivers;

class YclopsReactiveNavInterface: public mrpt::reactivenav::CReactiveInterfaceImplementation{
private:
	GPS* gps;
	Compass* compass;
	MotorCommand* motor;
	AbstractPoseEstimator* poseEst;
	mrpt::hwdrivers::CCameraSensor* camera;
	CPose3D* robotPose;

	float curV;
	float curW;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	YclopsReactiveNavInterface(mrpt::hwdrivers::CConfigFileBase&, string&);
	virtual ~YclopsReactiveNavInterface();
	bool getCurrentPoseAndSpeeds(mrpt::poses::CPose2D &curPose, float &curV, float &curW);
	bool changeSpeeds( float v, float w );
	bool senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles );
};

#endif /* YCLOPSREACTIVENAVINTERFACE_H_ */


