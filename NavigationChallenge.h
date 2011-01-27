/*
 * NavigationChallenge.h
 *
 *  Created on: Jan 25, 2011
 *      Author: igvcbyu
 */
#include <mrpt/utils.h>
#ifndef NAVIGATIONCHALLENGE_H_
#define NAVIGATIONCHALLENGE_H_


class NavigationChallenge{
public:
	void AutonomousMode(std::string& waypointsFile, bool challange);//0 autonomous 1 navigation
};

#endif /* NAVIGATIONCHALLENGE_H_ */
