/*
 * TSPSolver.cpp
 *
 *  Created on: Dec 11, 2010
 *      Author: tallred3
 */

#include "TSPSolver.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

TSPSolver::TSPSolver( double lat, double lon ) {
	this->visited.push_back(CPoint2D(lat,lon));
}

void TSPSolver::loadPoints(std::string filename) {

	CPoint2D tmp;
	ifstream datapointSet;
	datapointSet.open(filename.c_str(), ifstream::in);

	while( datapointSet.good() ) {
		datapointSet >> tmp.m_coords[LAT] >> tmp.m_coords[LON];
		toVisit.push_back(tmp);
	}

	datapointSet.close();

}

std::vector<mrpt::poses::CPoint2D> TSPSolver::solve() {
	TSPSolver::nieveTSPSolution( this->toVisit, this->visited );
	return this->visited;
}

void TSPSolver::nieveTSPSolution( std::vector<CPoint2D> & toVisit, std::vector<CPoint2D> & visited ) {

	while( toVisit.size() > 0 ) {
		double shortestDistance = 10000000000000;//just a really big number
		int closestIndex = 0;
		for( unsigned index = 0; index < toVisit.size(); ++index ) {
			double dist = haversineDistance( toVisit[index].m_coords[LAT], toVisit[index].m_coords[LON], visited.back().m_coords[LAT], visited.back().m_coords[LON] );
			if(shortestDistance > dist && dist > .1 ) {	//if the distance is really really small we must be checking the same point
				closestIndex = index;					//besides the gps isn't accurate to a tenth of a meter
				shortestDistance = dist;				//however there ought to be a better way then this and it should be fixed
			}
		}

		visited.push_back(toVisit[closestIndex]);
		toVisit.erase(toVisit.begin()+closestIndex);

	}
}

double TSPSolver::haversineDistance( double lat1, double lon1, double lat2, double lon2 ) {
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
