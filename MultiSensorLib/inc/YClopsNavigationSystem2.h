/**
 *  \file:YClopsNavigationSystem2.h
 *  \date: Feb 6, 2011
 *  \author: tallred3
 */

#ifndef YCLOPSNAVIGATIONSYSTEM2_H_
#define YCLOPSNAVIGATIONSYSTEM2_H_

#include "MotorCommandInterface.h"
#include "Compass.h"
#include <mrpt/utils/CConfigFile.h>

class YClopsNavigationSystem2 {
public:
	YClopsNavigationSystem2(mrpt::utils::CConfigFile & config);
	virtual ~YClopsNavigationSystem2();

	void doProcess();

	void setAutonomusMode();
	void setNavigationMode();
	void setIdle();

	bool toggleGpsDump();
	bool toggleCompassDump();
	bool toggleLidarDump();
	bool toggleCameraDump();

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

	MotorCommandInterface * motor;
	Compass * compass;
};

#endif /* YCLOPSNAVIGATIONSYSTEM2_H_ */
