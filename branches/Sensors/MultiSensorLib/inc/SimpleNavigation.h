/*
 * SimpleNavigation.h
 *
 *  Created on: Feb 1, 2011
 *      Author: quincyj2
 */

#ifndef SIMPLENAVIGATION_H_
#define SIMPLENAVIGATION_H_


#include "YclopsReactiveNavInterface.h"
#include <mrpt/base.h>
#include <mrpt/utils.h>

class SimpleNavigation {
private:
	string& fileName;
public:

	SimpleNavigation(string & fileName);
	virtual ~SimpleNavigation();

	void navigate(YclopsReactiveNavInterface& interface, mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams *navParams );
	void go();
};

#endif /* SIMPLENAVIGATION_H_ */
