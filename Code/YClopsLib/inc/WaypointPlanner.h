/**
 * @file WaypointPlanner.h
 * @brief Defines the classes relating to path planning
 * @date Dec 27, 2010
 * @author Thomas Eldon Allred
 */

#ifndef WAYPOINTPLANNER_H_
#define WAYPOINTPLANNER_H_

#include <string>
#include <vector>
#include <mrpt/poses/CPoint2D.h>

/**
 * @brief Abstract interface for path planning for the robot
 */
class AbstractNavigationInterface {
public:

	static const int LAT = 0; //!<Index for the Latitude in a CPoint2D
	static const int LON = 1; //!<Index for the Longitude in a CPoint2D

	/**
	 * @brief Abstract Navigation class constructor
	 * @param[in] lat representing the initial latitude
	 * @param[in] lon representing the initial longitude
	 */
	AbstractNavigationInterface( double lat, double lon );

	/**
	 * @brief Class destructor
	 */
	virtual ~AbstractNavigationInterface();

	/**
	 * @brief solve solves the navigation problem
	 * @return a list of gps weighpoints that need to be visited
	 * index 0 is the first then 1 then 2 ect.
	 */
	virtual mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t solve(bool inMeters) = 0;

	/**
	 * @brief changes a point from latitude and longitude to meters in x (positive direction is north) and y (positive direction is east) from the
	 * starting position
	 */
	void convertPointToMeters(mrpt::poses::CPoint2D& point);

	/**
	 * @brief reads in a stream of ascii whitespace delimited gps
	 * 		points from a file where the first one is the starting location and
	 * 		all subsequent are weighpoints to travel to.
	 * @param filename - the name of the raw gps point file
	 */
	void loadPoints(std::string filename, bool convertToMeters = false);

	/**
	 * @brief a function that computes the great circle distance of two points measured in degrees
	 * @returns The distance between two points in meters
	 */
	static double haversineDistance( double lat1, double lon1, double lat2, double lon2 );

	/**
	 * @brief a function that computes the bearing (angle) between two points measured in degrees
	 * @returns the bearing between two points in degrees
	 */
	static double calcBearing( double lat1, double lon1, double lat2, double lon2, bool isDegrees = false );

protected:
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t toVisit; //!< Vector of points that haven't been visited by the planning.
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t visited; //!< Vector of points that have been ordered.

};

/**
 * SequentialNavigation
 */
class SequentialNavigation: public AbstractNavigationInterface {
public:
	SequentialNavigation( double lat, double lon );
	virtual ~SequentialNavigation();

	/**
	 * SequentialNavigation::solve - returns the
	 * ordered list of navigation waypoints as they
	 * are ordered in the file
	 */
	 mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t solve(bool inMeters);

};

class TSPNavigation : public AbstractNavigationInterface{
public:

	TSPNavigation( double initLat, double initLon );
	virtual ~TSPNavigation() {};

	/**
	 * TSPNavigation::solve - solves the TSP by calling TSPNavigation::nieveTSPSolution
	 * @see nieveTSPSolution
	 */
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t solve(bool inMeters);

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
	static void nieveTSPSolution( mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
			mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited, bool inMeters );


	static void acoTSPSolution( mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit,
			mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & visited, bool inMeters);

};

#endif /* WAYPOINTPLANNER_H_ */
