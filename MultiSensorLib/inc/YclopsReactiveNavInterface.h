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

using namespace mrpt::hwdrivers;

class YclopsReactiveNavInterface: CReactiveInterfaceImplementation {
private:
	GPS& gps;
	Compass& compass;
	MotorCommand& motor;
	AbstractPoseEstimator& poseEst;
	mrpt::hwdrivers::CCameraSensor& camera;
	CPose3D& robotPose;

	float curV;
	float curW;
public:
	YclopsReactiveNavInterface(mrpt::hwdrivers::CConfigFileBase&);
	virtual ~YclopsReactiveNavInterface();
};

#endif /* YCLOPSREACTIVENAVINTERFACE_H_ */


