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
	//interface that will be used by the reactive nav to sense the enviroment and make the robot move
	YclopsReactiveNavInterface* interface = new YclopsReactiveNavInterface(motorControllerPort);
	//intial position
	mrpt::poses::CPose2D pose = CPoint2D();
	float v = 0;
	float w = 0;


	interface->getCurrentPoseAndSpeeds(pose, v,w);
	double lat = pose.x();
	double lon = pose.y();

	// solves the tsp problem, it autonomous challenge we need to add some code here to make it work.
	TSPNavigation nav = TSPNavigation(lat,lon);

	nav.loadPoints(waypointsFile);
	//waypoints in the order we want to visit them
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t points = nav.solve();

	//the reactive nav to get from the current position to the next point the tsp solver wants us to go to
	mrpt::reactivenav::YclopsNavigationSystem reactivenav = mrpt::reactivenav::YclopsNavigationSystem(*interface, false, false);

	mrpt::utils::CConfigFile navConfig("robot.ini");
	mrpt::utils::CConfigFile robotConfig("robotConf.ini");

	reactivenav.loadConfigFile(navConfig,robotConfig);

	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator iter = points.begin();
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator end = points.end();

	//go until there aren't anymore waypoints
	while (iter != end)
	{
		//setting up the next waypoint
		mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams   navParams;
		navParams.target.x = iter->x();
		navParams.target.y = iter->y();
		//distance from the waypoint in meters before the robot considers itself at the point
		navParams.targetAllowedDistance = challange ?  0.20f: 7.0f;
		navParams.targetIsRelative = !challange;

		//gives the reactive nav the next goal
		reactivenav.navigate( &navParams );
		//go until it makes it to the point
		while(reactivenav.getCurrentState() ==
				mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING)
		{
			reactivenav.navigationStep();
		}
		++iter;
	}
	//time for a vitory dance
}
