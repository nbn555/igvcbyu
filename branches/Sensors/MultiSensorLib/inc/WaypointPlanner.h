/*
 * AbstractNavigationInterface.h
 *
 *  Created on: Dec 27, 2010
 *      Author: tallred3
 */

#ifndef WAYPOINTPLANNER_H_
#define WAYPOINTPLANNER_H_

#include <string>
#include <vector>
#include <mrpt/poses/CPoint2D.h>

class AbstractNavigationInterface {
public:

	static const int LAT = 0;
	static const int LON = 1;

	AbstractNavigationInterface( double lat, double lon );
	virtual ~AbstractNavigationInterface();

	/**
	 * solve - solves the navigation problem
	 * @return - a list of gps weighpoints that need to be visited
	 * index 0 is the first then 1 then 2 ect.
	 */
	virtual std::vector<mrpt::poses::CPoint2D> solve() = 0;

	/**
	 * loadRawGPSPointFile - reads in a stream of ascii whitespace delimited gps
	 * 		points from a file where the first one is the starting location and
	 * 		all subsequent are weighpoints to travel to.
	 * @param filename - the name of the raw gps point file
	 */
	void loadPoints(std::string filename);

	/**
	 * "Haver Sine" Distance - is a function that computes the great circle distance of two points measured in degrees
	 * @returns The distance between two points in meters
	 */
	static double haversineDistance( double lat1, double lon1, double lat2, double lon2 );

	/**
	 * Bearing Calculator - is a function that computes the bearing (angle) between two points measured in degrees
	 * @returns the bearing between two points in degrees
	 */
	static double calcBearing( double lat1, double lon1, double lat2, double lon2 );

protected:
	std::vector<mrpt::poses::CPoint2D> toVisit;
	std::vector<mrpt::poses::CPoint2D> visited;

};

class SequentialNavigation: public AbstractNavigationInterface {
public:
	SequentialNavigation( double lat, double lon );
	virtual ~SequentialNavigation();

	/**
	 * SequentialNavigation::solve - returns the
	 * ordered list of navigation waypoints as they
	 * are ordered in the file
	 */
	std::vector<mrpt::poses::CPoint2D> solve();

};

class TSPNavigation : public AbstractNavigationInterface{
public:

	TSPNavigation( double initLat, double initLon );
	virtual ~TSPNavigation() {};

	/**
	 * TSPNavigation::solve - solves the TSP by calling TSPNavigation::nieveTSPSolution
	 * @see nieveTSPSolution
	 */
	std::vector<mrpt::poses::CPoint2D> solve();

private:

	/**
	 * nieveTSPSolution - a nieve solution to the traveling salesman problem
	 * 		this implementation is greedy in that it simply goes to the nearest
	 * 		unvisited point
	 * @param toVisit a std::vector of latitudes and longitudes that are to be visited
	 * @param visited a std::vector of (initially) one latitude and longitude
	 * 		representing where we start after the algorithm is run it will be an ordered
	 * 		set of latlon_t where the first is where we are at and each subsequent
	 * 		latlon_t is the next closest latlon_t
	 */
	static void nieveTSPSolution( std::vector<mrpt::poses::CPoint2D> & toVisit, std::vector<mrpt::poses::CPoint2D> & visited );

};

#endif /* WAYPOINTPLANNER_H_ */