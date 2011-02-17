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
	mrpt::reactivenav::CReactiveInterfaceImplementation* interface;
public:

	SimpleNavigation(string & fileName, mrpt::reactivenav::CReactiveInterfaceImplementation* interface);
	virtual ~SimpleNavigation();
	void setFileName(string & fileName);
	void navigate(mrpt::reactivenav::CReactiveInterfaceImplementation& interface, mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams *navParams );
	void go();
};

#endif /* SIMPLENAVIGATION_H_ */
