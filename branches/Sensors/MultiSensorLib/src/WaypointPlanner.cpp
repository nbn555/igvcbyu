/*
 * AbstractNavigationInterface.cpp
 *
 *  Created on: Dec 27, 2010
 *      Author: tallred3
 */

#include "WaypointPlanner.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

AbstractNavigationInterface::AbstractNavigationInterface( double lat, double lon ) {
	this->visited.push_back(CPoint2D(lat,lon));
}

AbstractNavigationInterface::~AbstractNavigationInterface() {
	// TODO Auto-generated destructor stub
}

void AbstractNavigationInterface::loadPoints(std::string filename) {

	CPoint2D tmp;
	ifstream datapointSet;
	datapointSet.open(filename.c_str(), ifstream::in);

	while( datapointSet.good() ) {
		datapointSet >> tmp.m_coords[LAT] >> tmp.m_coords[LON];
		toVisit.push_back(tmp);
	}

	datapointSet.close();

}

double AbstractNavigationInterface::haversineDistance( double lat1, double lon1, double lat2, double lon2 ) {
	const double EARTH_RADIUS_AVERAGE = 6371009;//average earth radius in meters

	double deltalat = lat2 - lat1;
	double deltalon = lon2 - lon1;

	//convert from degree to radian
	deltalat *= M_PI/180.0;
	deltalon *= M_PI/180.0;
	lat1 *= M_PI/180.0;
	lat2 *= M_PI/180.0;

	//nasty spherical trig stuff
	double a = pow( sin( deltalat / 2 ), 2 ) +
			cos(lat1) * cos(lat2) * pow( sin( deltalon / 2 ), 2 );
	double c = 2 * atan2( sqrt(a), sqrt(1-a) );

	return EARTH_RADIUS_AVERAGE * c;
}

double AbstractNavigationInterface::calcBearing( double lat1, double lon1, double lat2, double lon2 ) {
	//convert inputs to radians
	lat1 *= M_PI/180.0;
	lon1 *= M_PI/180.0;
	lat2 *= M_PI/180.0;
	lon2 *= M_PI/180.0;
	double radBearing;
	double t1, t2, t3, t4, t5;

	//Do a bunch of magic trig
	t1 = sin(lat1) * sin(lat2);
	t2 = cos(lat1) * cos(lat2);
	t3 = cos(lon1 - lon2);
	t4 = t2 * t3;
	t5 = t1 + t4;
	double rad_dist = atan(-t5/sqrt(-t5*t5+1))+2*atan(1);
	t1 = sin(lat2) - sin(lat1) * cos(rad_dist);
	t2 = cos(lat1) * sin(rad_dist);
	t3 = t1/t2;

	if(sin(lon2 - lon1) < 0 ) {
		t4 = atan(-t3/sqrt(-t3*t3+1)) + 2 * atan(1);
		radBearing = t4;
	} else {
		t4 = -t3 * t3 + 1;
		t5 = 2 * M_PI - atan(-t3 / sqrt(-t3 * t3 + 1)) + 2 * atan(1);
		radBearing = t5;
	}
	//convert outputs to degrees
	radBearing *= 180.0/M_PI;
	return radBearing;
}

SequentialNavigation::SequentialNavigation( double lat, double lon ): AbstractNavigationInterface( lat, lon ) { }

SequentialNavigation::~SequentialNavigation() { }

std::vector<mrpt::poses::CPoint2D, Eigen::aligned_allocator<mrpt::poses::CPoint2D> > SequentialNavigation::solve(){
	this->visited.insert(this->visited.end(), this->toVisit.begin(),this->toVisit.end());
	return this->visited;
}

TSPNavigation::TSPNavigation( double lat, double lon ): AbstractNavigationInterface(lat, lon) { }

std::vector<mrpt::poses::CPoint2D, Eigen::aligned_allocator<mrpt::poses::CPoint2D> > TSPNavigation::solve() {
	TSPNavigation::nieveTSPSolution( this->toVisit, this->visited );
	return this->visited;
}

void TSPNavigation::nieveTSPSolution(  mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
		 mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited ) {

	while( toVisit.size() > 0 ) {
		double shortestDistance = 10000000000000.0;//just a really big number
		int closestIndex = 0;
		for( unsigned index = 0; index < toVisit.size(); ++index ) {
			double dist = haversineDistance( toVisit[index].m_coords[LAT], toVisit[index].m_coords[LON], visited.back().m_coords[LAT], visited.back().m_coords[LON] );
			if(shortestDistance > dist && dist > .1 ) {	//if the distance is really really small we must be checking the same point
				closestIndex = index;					//besides the gps isn't accurate to a tenth of a meter
				shortestDistance = dist;				//however there ought to be a better way then this and it should be fixed
			}
		}
		mrpt::poses::CPoint2D& point = (toVisit[closestIndex]);
		int multx = point.m_coords[LON] > visited.front().m_coords[LON] ? 1 : -1;
		int multy = point.m_coords[LAT] > visited.front().m_coords[LAT] ? 1 : -1;
		point.m_coords[LAT] = multy * (haversineDistance( visited.front().m_coords[LAT],toVisit[closestIndex].m_coords[LON],
				toVisit[closestIndex].m_coords[LAT], toVisit[closestIndex].m_coords[LON] ));
		point.m_coords[LON] = multx * (haversineDistance( toVisit[closestIndex].m_coords[LAT], visited.front().m_coords[LON],
				toVisit[closestIndex].m_coords[LAT], toVisit[closestIndex].m_coords[LON] ));

		visited.push_back(toVisit[closestIndex]);
		toVisit.erase(toVisit.begin()+closestIndex);

	}
	visited.front().m_coords[LAT] = 0;
	visited.front().m_coords[LON] = 0;
}
