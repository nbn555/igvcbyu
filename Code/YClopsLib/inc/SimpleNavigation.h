/**
 * @file SimpleNavigation.h
 * @date Feb 1, 2011
 * @author quincyj2
 * @brief Simple Navigation Header
 */

#ifndef SIMPLENAVIGATION_H_
#define SIMPLENAVIGATION_H_


#include "YClopsReactiveNavInterface.h"
#include <mrpt/base.h>
#include <mrpt/utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

class SimpleNavigation {
private:
	std::string fileName;
	bool inMeters;
	mrpt::reactivenav::CReactiveInterfaceImplementation* interface;
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t points;
	mrpt::reactivenav::CAbstractReactiveNavigationSystem::TState state;
	mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams* navParams;
	void gotoNextPoint();
	float distance(mrpt::poses::CPose2D& curPose)
	{
		float x1 = curPose.x();
		float y1 = curPose.y();
		float x2 = navParams->target.x;
		float y2 = navParams->target.y;
		double dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
		cout << y1 << " " << x1 << " is a distance of " << dist << " from " << y2 << " " << x2 << endl;
		return dist;
	}

	double calcBearing(mrpt::poses::CPose2D& curPose)
	{
		float x1 = curPose.x();
		float y1 = curPose.y();
		float x2 = navParams->target.x;
		float y2 = navParams->target.y;
		float dx = x2 - x1;
		float dy = y2 - y1;
		return atan2(dy,dx);
	}
public:

	SimpleNavigation(std::string & fileName, mrpt::reactivenav::CReactiveInterfaceImplementation* interface);
	virtual ~SimpleNavigation();
	void setFileName(std::string & fileName, bool inMeters);
	void navigate( mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams *navParams );
	void go();
	void navigationStep();
	void loadConfigFile(mrpt::utils::CConfigFileBase& interfaceConfig, mrpt::utils::CConfigFileBase& robotConfig){ ; }
	mrpt::reactivenav::CAbstractReactiveNavigationSystem::TState getCurrentState(){ return state;}
};

#endif /* SIMPLENAVIGATION_H_ */
