/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <vector>

#include <mrpt/poses/CPoint2D.h>

#include "TSPSolver.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

/**
 * A test driver for the TSPSolver which is a class that solves the traveling sales man problem
 */
int main( int argc, char** argv ) {

	TSPSolver solver(40.247605, -111.648192);

	solver.loadPoints("gpsWeighpoints_raw.txt");

	vector<CPoint2D> visited = solver.solve();

	cout.precision(10);
	for(vector<CPoint2D>::iterator point = visited.begin(); point != visited.end(); ++point )
		cout << point->m_coords[TSPSolver::LAT] << " " << point->m_coords[TSPSolver::LON] << endl;

	return 0;
}
