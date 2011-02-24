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
	// TODO Auto-generated destructor stub
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

	/*
	 * I added this because I changed the motor controller so it requires a config file
	 * for initialization.  CConfigFileMemory is a psudo config file so the motor controller
	 * code should still work with SimpleNavigation.  All the writes are required values for the
	 * motor controller.  Though this should really be brought in from a config file for the motor
	 * controller.  Config needs to be a pointer.  Eldon
	 */
	mrpt::utils::CConfigFileMemory * config = new mrpt::utils::CConfigFileMemory();
	config->write("MOTOR","COM_port_LIN","/dev/ttyS1");
	config->write("MOTOR", "MAX_FORWARD_LEFT", 400 );
	config->write("MOTOR", "MAX_FORWARD_RIGHT", 400 );
	config->write("MOTOR", "MAX_REVERSE_LEFT", -400 );
	config->write("MOTOR", "MAX_REVERSE_RIGHT", -400 );
	MotorController::setConfigFile(config);


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
	cout << "Yaw: " << yaw << endl;
	wantedYaw = calcBearing(curPose);//AbstractNavigationInterface::calcBearing(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
	cout << "Wanted Yaw: " << wantedYaw << endl;
	cout << "Difference: " << abs(yaw-wantedYaw) << endl;
	double dist = distance(curPose);
	if(!dist > navParams->targetAllowedDistance)
	{
		cout << "Reached Way Point" << endl;
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
	if(abs((yaw-wantedYaw)) > .2)
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
	}
	else
	{
		double dist = distance(curPose);
		cout << "\tDriving Towards Target. Remaining Distance: " << dist << endl;//AbstractNavigationInterface::haversineDistance(curPose.x(), curPose.y(), navParams->target.x, navParams->target.y);
		if(dist > navParams->targetAllowedDistance)
		{
			interface->changeSpeeds(1.4,0);
			cout << "\tDriving Towards Target. Remaining Distance: " << dist << endl;

		}
		else
		{
			cout << "Reached Way Point" << endl;
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
