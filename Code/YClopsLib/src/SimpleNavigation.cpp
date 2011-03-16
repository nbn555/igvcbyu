/*
 * SimpleNavigation.cpp
 *
 *  Created on: Feb 1, 2011
 *      Author: quincyj2
 */



#include "WaypointPlanner.h"

#include "SimpleNavigation.h"

#include <mrpt/utils/CConfigFileMemory.h>

SimpleNavigation::SimpleNavigation(string & fileName, mrpt::reactivenav::CReactiveInterfaceImplementation* interface): fileName(fileName), interface(interface) {
}

SimpleNavigation::~SimpleNavigation() {
	delete interface;
}

void SimpleNavigation::setFileName(string & fileName, bool inMeters)
{
	cout << fileName << endl;
	this->fileName = fileName;
	this->inMeters = inMeters;
}

void SimpleNavigation::go()
{

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

	nav.loadPoints(fileName, !inMeters);
	//waypoints in the order we want to visit them
	 points =  nav.solve(false);

	 gotoNextPoint();


}
void SimpleNavigation::gotoNextPoint()
{
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator iter = points.begin();


	//setting up the next waypoint
	mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams*   navParams = new mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams();
	navParams->target.x = iter->x();
	navParams->target.y = iter->y();
	//distance from the waypoint in meters before the robot considers itself at the point
	navParams->targetAllowedDistance = .70f;
	//navParams.targetIsRelative = !challange; is being ignored
	//gives the reactive nav the next goal
	navigate(navParams );
}
void SimpleNavigation::navigate(mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams *navParam )
{
	if(points.size() == 0)
	{
		mrpt::poses::CPoint2D* point = new CPoint2D();
		point->x(navParam->target.x);
		point->y(navParam->target.y);
		points.push_back(*point);
	}
	state = mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING;

	this->navParams = new mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams();
	navParams->target.x = navParam->target.x;
	navParams->target.y = navParam->target.y;
	navParams->targetAllowedDistance = navParam->targetAllowedDistance;

}
void SimpleNavigation::navigationStep()
{
	mrpt::poses::CPose2D curPose;
	double yaw;
	double wantedYaw;
	float cur_w;
		float cur_v;
	interface->getCurrentPoseAndSpeeds(curPose, cur_v, cur_w);
	yaw = curPose.phi();//Robotpose returns yaw in radians
	LOG_AI(DEBUG4) << "Simple Nav Yaw: " << yaw << endl;
	wantedYaw = calcBearing(curPose);//AbstractNavigationInterface::calcBearing(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
	LOG_AI(DEBUG4) << "Simple Nav Wanted Yaw: " << wantedYaw << endl;
	double dist = distance(curPose);
	double diffrence = wantedYaw - yaw;
	double minDiff = diffrence;
	if(diffrence > M_PI)
	{
		minDiff = 2*M_PI - diffrence;
	}
	else if(diffrence < -M_PI)
	{
		minDiff = 2*M_PI + diffrence;
	}

	LOG_AI(DEBUG4) << "Simple Nav Angular Difference: " << minDiff << endl;

	if(!dist > navParams->targetAllowedDistance)
	{
		LOG_AI(INFO) << "Reached Way Point" << endl;
		points.erase(points.begin());
		if(points.size() != 0)//this->points)
		{
			gotoNextPoint();
		}
		else
		{
			state = mrpt::reactivenav::CAbstractReactiveNavigationSystem::IDLE;
		}
	}
	if(abs((minDiff)) > .2)
	{
		bool turnRight = minDiff > 0;
		if(turnRight)
		{
			LOG_AI(DEBUG) << "\tTurning Right" << endl;
			interface->changeSpeeds(.5f,2);
		}
		else
		{
			LOG_AI(DEBUG) << "\tTurning Left" << endl;
			interface->changeSpeeds(.5f, -2);
		}
	}
	else
	{
		double dist = distance(curPose);
		LOG_AI(DEBUG) << "\tDriving Towards Target. Remaining Distance: " << dist << endl;//AbstractNavigationInterface::haversineDistance(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
		if(dist > navParams->targetAllowedDistance)
		{
			interface->changeSpeeds(1.4,0);
			LOG_AI(DEBUG) << "\tDriving Towards Target. Remaining Distance: " << dist << endl;

		}
		else
		{
			LOG_AI(INFO) << "Reached Way Point" << endl;
			points.erase(points.begin());
			if(points.size() != 0)//this->points)
			{
				gotoNextPoint();
			}
			else
			{
				state = mrpt::reactivenav::CAbstractReactiveNavigationSystem::IDLE;
			}
		}
	}
}
