/*
 * SimpleNavigation.cpp
 *
 *  Created on: Feb 1, 2011
 *      Author: quincyj2
 */



#include "WaypointPlanner.h"

#include "SimpleNavigation.h"

SimpleNavigation::SimpleNavigation(string & fileName, mrpt::reactivenav::CReactiveInterfaceImplementation* interface): fileName(fileName), interface(interface) {
}

SimpleNavigation::~SimpleNavigation() {
	// TODO Auto-generated destructor stub
	delete interface;
}

void SimpleNavigation::setFileName(string & fileName)
{
	this->fileName = fileName;
}

void SimpleNavigation::go()
{
	MotorController::setPortName("ttyS0");
	//interface that will be used by the reactive nav to sense the environment and make the robot move

	//initial position
	mrpt::poses::CPose2D pose = CPoint2D();
	float v = 0;
	float w = 0;

	interface->getCurrentPoseAndSpeeds(pose, v,w);

	double lat = pose.x();
	double lon = pose.y();

	// solves the tsp problem, in autonomous challenge we need to add some code here to make it work.
	TSPNavigation nav = TSPNavigation(lat,lon);

	nav.loadPoints(fileName);
	//waypoints in the order we want to visit them
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t points = nav.solve(false);



	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator iter = points.begin();
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator end = points.end();
		//go until there aren't anymore waypoints
	double count = 0;
	while (iter != end)
		{
			cout << "Way point # " << count << endl;
			//setting up the next waypoint
			mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams   navParams;
			navParams.target.x = iter->x();
			navParams.target.y = iter->y();
			//distance from the waypoint in meters before the robot considers itself at the point
			navParams.targetAllowedDistance = 0.20f;
			//navParams.targetIsRelative = !challange; is being ignored

			//gives the reactive nav the next goal
			navigate(&navParams );
			/*//go until it makes it to the point
			while(reactivenav.getCurrentState() ==
					mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING)
			{
				reactivenav.navigationStep();
			}*/
			++count;
			++iter;

		}
}

void SimpleNavigation::navigate(mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams *navParams )
{
	mrpt::poses::CPose2D curPose;
	float cur_w;
	float cur_v;
	cout << "Navigating To: lat " << navParams->target.x << " lon " << navParams->target.y << endl;
	interface->getCurrentPoseAndSpeeds(curPose, cur_v, cur_w);
	double yaw = curPose.phi();
	double wantedYaw = AbstractNavigationInterface::calcBearing(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
	float distance = AbstractNavigationInterface::haversineDistance(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
	if(distance > navParams->targetAllowedDistance)
	{
		while(abs((yaw-wantedYaw)/wantedYaw) > .05)
		{
			bool turnRight = yaw < wantedYaw;
			if(turnRight)
			{
				cout << "\tTurning Right" << endl;
				interface->changeSpeeds(.5f,2);
			}
		else
			{
				cout << "\tTurning Left" << endl;
				interface->changeSpeeds(.5f, -2);
			}

			usleep(100000);
			interface->getCurrentPoseAndSpeeds(curPose, cur_v, cur_w);
			yaw = curPose.phi();//Robotpose returns yaw in radians
			cout << "Yaw: " << yaw << endl;
			wantedYaw = AbstractNavigationInterface::calcBearing(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
			cout << "Wanted Yaw: " << wantedYaw << endl;
			cout << "Difference: " << abs(yaw-wantedYaw) << endl;
		}
		distance = AbstractNavigationInterface::haversineDistance(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
		while(distance > navParams->targetAllowedDistance)
		{
			interface->changeSpeeds(1.4,0);
			cout << "\tDriving Towards Target. Remaining Distance: " << distance << endl;
			usleep(100000);
			distance = AbstractNavigationInterface::haversineDistance(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
		}
	}
	cout << "Reached Way Point" << endl;
}
