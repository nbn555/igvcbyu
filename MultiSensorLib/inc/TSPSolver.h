/*
 * TSPSolver.h
 *
 *  Created on: Dec 11, 2010
 *      Author: tallred3
 */

#ifndef TSPSOLVER_H_
#define TSPSOLVER_H_

#include <string>
#include <mrpt/poses/CPoint2D.h>

class TSPSolver {
public:

	static const int LAT = 0;
	static const int LON = 1;

	TSPSolver( double initLat, double initLon );
	virtual ~TSPSolver() {};

	/**
	 * loadRawGPSPointFile - reads in a stream of ascii whitespace delimited gps
	 * 		points from a file where the first one is the starting location and
	 * 		all subsequent are weighpoints to travel to.
	 * @param filename - the name of the raw gps point file
	 */
	void loadPoints(std::string filename);
	std::vector<mrpt::poses::CPoint2D> solve();

	/**
	 * "Haver Sine" Distance - is a function that computes the great circle distance of two points
	 * @returns The distance between two points in meters
	 */
	static double haversineDistance( double lat1, double lon1, double lat2, double lon2 );

	/**
	 * Bearing Calculator - is a function that computes the bearing (angle) between two points
	 * @returns the bearing between two points in degrees
	 */
	static double bearingCalculator( double lat1, double lon1, double lat2, double lon2 );

private:
	std::vector<mrpt::poses::CPoint2D> toVisit;
	std::vector<mrpt::poses::CPoint2D> visited;

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

#endif /* TSPSOLVER_H_ */
