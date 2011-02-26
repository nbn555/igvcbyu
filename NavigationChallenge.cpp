/*
 * NavigationChallenge.cpp
 *
 *  Created on: Jan 25, 2011
 *      Author: igvcbyu
 */

#include "NavigationChallenge.h"
#include "YClopsReactiveNavInterface.h"
#include "WaypointPlanner.h"
#include "MotorController.h"
#include <mrpt/base.h>
#include <mrpt/utils.h>
#include "YclopsNavigationSystem.h"
#include <string>

using namespace mrpt;
using namespace mrpt::poses;
using namespace std;

void NavigationChallenge::AutonomousMode(string& waypointsFile, bool challange)
{
	mrpt::utils::CConfigFileMemory * config = new mrpt::utils::CConfigFileMemory();
	config->write("MOTOR","COM_port_LIN","/dev/ttyS1");
	config->write("MOTOR", "MAX_FORWARD_LEFT", 400 );
	config->write("MOTOR", "MAX_FORWARD_RIGHT", 400 );
	config->write("MOTOR", "MAX_REVERSE_LEFT", -400 );
	config->write("MOTOR", "MAX_REVERSE_RIGHT", -400 );
	MotorController::setConfigFile(config);

	//interface that will be used by the reactive nav to sense the environment and make the robot move
	YClopsReactiveNavInterface* interface = new YClopsReactiveNavInterface();
	//initial position
	mrpt::poses::CPose2D pose = CPose2D();
	float v = 0;
	float w = 0;


	interface->getCurrentPoseAndSpeeds(pose, v,w);
	double lat = pose.x();
	double lon = pose.y();

	cout << "lat: " << lat << " lon: " << endl;
	// solves the tsp problem, it autonomous challenge we need to add some code here to make it work.
	TSPNavigation nav = TSPNavigation(lat,lon);

	nav.loadPoints(waypointsFile);
	//waypoints in the order we want to visit them
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t points = nav.solve(true);

	cout << "TSP Solved" << endl;
	//the reactive nav to get from the current position to the next point the tsp solver wants us to go to
	mrpt::reactivenav::YclopsNavigationSystem reactivenav = mrpt::reactivenav::YclopsNavigationSystem(*interface, true, true);

	mrpt::utils::CConfigFile navConfig("robot.ini");
	mrpt::utils::CConfigFile robotConfig("robotConf.ini");

	reactivenav.loadConfigFile(navConfig,robotConfig);

	cout << "Reactive nav intialized" << endl;

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

		cout << "navigating to " << navParams.target.x << " " << navParams.target.y << endl;
		//gives the reactive nav the next goal
		reactivenav.navigate( &navParams );
		while(reactivenav.getCurrentState() !=
						mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING);
		//go until it makes it to the point
			while(reactivenav.getCurrentState() ==
							mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING)
		{
			reactivenav.navigationStep();
		}
		++iter;
	}
	//time for a victory dance
	for(int i = 0; i < 100; i++)
		{
			float v = .2;
			float w = 40;
			float time = 100000;
			interface->changeSpeeds(v,w);
			usleep(time);
		}

}
