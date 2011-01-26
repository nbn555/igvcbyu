/*
 * NavigationChallenge.cpp
 *
 *  Created on: Jan 25, 2011
 *      Author: igvcbyu
 */

#include "NavigationChallenge.h"
#include "YclopsReactiveNavInterface.h"
#include "WaypointPlanner.h"
#include <mrpt/base.h>
#include <mrpt/utils.h>
#include "../YclopsNavigationSystem.h"

void NavigationChallenge::AutonomousMode(string& waypointsFile, bool challange)
{
	string motorControllerPort = "ttyS000";
	YclopsReactiveNavInterface* interface = new YclopsReactiveNavInterface(motorControllerPort);

	mrpt::poses::CPose2D pose = CPoint2D();
	float v = 0;
	float w = 0;

	interface->getCurrentPoseAndSpeeds(pose, v,w);
	double lat = pose.x();
	double lon = pose.y();

	TSPNavigation nav = TSPNavigation(lat,lon);

	nav.loadPoints(waypointsFile);
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t points = nav.solve();

	mrpt::reactivenav::YclopsNavigationSystem reactivenav = mrpt::reactivenav::YclopsNavigationSystem(*interface, false, false);

	mrpt::utils::CConfigFile navConfig("robot.ini");
	mrpt::utils::CConfigFile robotConfig("robotConf.ini");

	reactivenav.loadConfigFile(navConfig,robotConfig);

	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator iter = points.begin();
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator end = points.end();


	while (iter != end)
	{
		mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams   navParams;
		navParams.target.x = iter->x();
		navParams.target.y = iter->y();
		navParams.targetAllowedDistance = challange ?  0.40f: 20.0f;
		navParams.targetIsRelative = challange;

		reactivenav.navigate( &navParams );

		while(reactivenav.getCurrentState() ==
				mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING)
		{
			reactivenav.navigationStep();
		}
		++iter;
	}

}
