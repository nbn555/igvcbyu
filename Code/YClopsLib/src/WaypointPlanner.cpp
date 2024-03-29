/*
 * AbstractNavigationInterface.cpp
 *
 *  Created on: Dec 27, 2010
 *      Author: tallred3
 */

#include "WaypointPlanner.h"
#include "TSPSolver.h"
#include "Tour.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;
using namespace lkinhou;

AbstractNavigationInterface::AbstractNavigationInterface( double lat, double lon ) {
	this->visited.push_back(CPoint2D(lat,lon));
}

AbstractNavigationInterface::~AbstractNavigationInterface() {
}
void AbstractNavigationInterface::convertPointToMeters(CPoint2D& point)
{
	double length = haversineDistance(this->visited.front().m_coords[LAT], this->visited.front().m_coords[LON],
			point.m_coords[LAT], point.m_coords[LON]);
	double direction = calcBearing(this->visited.front().m_coords[LAT], this->visited.front().m_coords[LON],
			point.m_coords[LAT], point.m_coords[LON]);
	point.m_coords[LAT] = sin(direction)*length;
	point.m_coords[LON] = -cos(direction)*length;
}
void AbstractNavigationInterface::loadPoints(std::string filename, bool convertToMeters) {

	CPoint2D tmp;
	ifstream datapointSet;
	datapointSet.open(filename.c_str(), ifstream::in);

	while( datapointSet.good() ) {
		datapointSet >> tmp.m_coords[LAT] >> tmp.m_coords[LON];
		if(convertToMeters)
		{
			convertPointToMeters(tmp);
		}
		else
		{
			//double temp_d = tmp.m_coords[LAT];
			//tmp.m_coords[LAT] = tmp.m_coords[LON];
			//tmp.m_coords[LON] = temp_d;
		}
		if(datapointSet.good())
		{
			cout << "adding point"<<endl;
			toVisit.push_back(tmp);
		}
	}
	if(convertToMeters)
	{
		visited.front().m_coords[LAT] = 0;
		visited.front().m_coords[LON] = 0;
	}
	datapointSet.close();
	for(unsigned int i = 0 ; i < toVisit.size(); i++) {
		cout << "after load " <<toVisit[i]<< endl;
	}

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

double AbstractNavigationInterface::calcBearing( double lat1, double lon1, double lat2, double lon2, bool isDegrees ) {
	double deltalat = lat2 - lat1;
		double deltalon = lon2 - lon1;

		//convert from degree to radian
		deltalat *= M_PI/180.0;
		deltalon *= M_PI/180.0;
	lat1 *= M_PI/180.0;
	lon1 *= M_PI/180.0;
	lat2 *= M_PI/180.0;
	lon2 *= M_PI/180.0;
	//atan2( 	sin(Δlong).cos(lat2),cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong) )
	double radBearing = atan2(sin(deltalon)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalon));

	if( isDegrees )	radBearing *= 180.0/M_PI;
	return radBearing;
}

SequentialNavigation::SequentialNavigation( double lat, double lon ): AbstractNavigationInterface( lat, lon ) { }

SequentialNavigation::~SequentialNavigation() { }

std::vector<mrpt::poses::CPoint2D, Eigen::aligned_allocator<mrpt::poses::CPoint2D> > SequentialNavigation::solve(bool inMeters){

	this->visited.insert(this->visited.end(), this->toVisit.begin(),this->toVisit.end());
	/*if(inMeters)
	{
		//!@TODO convert all points to meters from start point
	}*/
	this->visited.push_back(this->visited.front());
	visited.erase(visited.begin());
	return this->visited;
}

TSPNavigation::TSPNavigation( double lat, double lon ): AbstractNavigationInterface(lat, lon) { }

std::vector<mrpt::poses::CPoint2D, Eigen::aligned_allocator<mrpt::poses::CPoint2D> > TSPNavigation::solve(bool inMeters) {
	TSPNavigation::nieveTSPSolution( this->toVisit, this->visited , inMeters);
	//TSPNavigation::acoTSPSolution( this->toVisit, this->visited, inMeters);

	cout << "toVisit input: " << endl;
	for(unsigned int i = 0 ; i < this->toVisit.size(); i++)
	{
		cout << this->toVisit[i]<< endl;
	}
	cout << "Expected output: " << endl;
	for(unsigned int i = 0 ; i < this->visited.size(); i++)
	{
		cout << this->visited[i]<< endl;
	}
	return this->visited;
}

void TSPNavigation::nieveTSPSolution(  mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
		 mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited , bool inMeters) {
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
		visited.push_back(toVisit[closestIndex]);
		toVisit.erase(toVisit.begin()+closestIndex);


	}
	/*if(inMeters){
		visited.front().m_coords[LAT] = 0;
		visited.front().m_coords[LON] = 0;
	}*/
	mrpt::poses::CPoint2D finish = mrpt::poses::CPoint2D();
	finish.m_coords[LAT] = visited.front().m_coords[LAT];
	finish.m_coords[LON] = visited.front().m_coords[LON];
	visited.push_back(finish);
	visited.erase(visited.begin());
}

void convertInput(const mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
		vector<City> & inputCities){
	inputCities.clear();
	for(int i=0; i < toVisit.size(); i++){
		City c;
		//c.x = toVisit[i].m_coords[LAT];
		c.x = toVisit[i].m_coords[0];
		//c.y = toVisit[i].m_coords[LON];
		c.y = toVisit[i].m_coords[1];
		inputCities.push_back(c);
	}
}

/*
void convertOutput( mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
		mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited,
		Tour & bestTour){

	visited.clear();
	for (int t=0; t < toVisit.size(); t++){
		visited.push_back(toVisit[bestTour.getCity(t)]);
	}
}
*/
void TSPNavigation::acoTSPSolution( mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
		mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited , bool inMeters){

	// convert vectors of CPoint2D to vector<City> inCities;
	vector<City> inputCities;
	convertInput(toVisit, inputCities);

	// solve it
	TSPSolver solver(inputCities, inMeters);
	solver.solveIt();

	// get the best tour
	Tour bestTour = solver.getBestTour();

	// convert it back to  vectors of CPoint2D
	//convertOutput(visited, bestTour);
	visited.clear();
	for (int t=0; t < toVisit.size(); t++){
		visited.push_back(toVisit[bestTour.getCity(t)]);
	}
}
