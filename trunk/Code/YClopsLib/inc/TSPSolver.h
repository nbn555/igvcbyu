/*
 * TSPSolver.h
 *
 *  Created on: Mar 1, 2011
 *      Author: Tobias
 */

#include <vector>
#include "Tour.h"

#ifndef TSPSOLVER_H_
#define TSPSOLVER_H_

using namespace std;
namespace lkinhou {

struct City {
	int id;
	double x;
	double y;
};

class TSPSolver {
private:
	static const bool DEBUG = false;

	// updated in parentSelection method
	Tour _curBestTour;

	bool _inMeters;
	int _numPopulation;
	int _numCity;
	int _numParents;
	int _stopK;
	int _changeCount;
	int _numSwapCityMutation;
	// The population
	vector<Tour> _population;

	// The chosen parents in p
	vector<Tour> _parents;

	// Cities
	vector<City> inCities;

	// GetRandomInteger
	// lr: left range
	// rr: right range
	// return an random integer in [lr,rr]
	int getRandomInteger(int lr, int rr);

	// Initialization
	// Randomly generate _numPopulation valid routes
	void initialization();

	// GetDist
	// a: a city
	// b: another city
	// return the display between these two cities
	double getDist(const City & a, const City & b);

	// Evaluation
	// Calculating the cost of each tour in the population
	// And save the best tour so far
	void evaluation();

	// ParentSelection
	// Select _numParents parents in the population
	void parentSelection();

	// Swap a and b
	void swap(int & a, int & b);

	// Crossover
	// Combine parents by swapping the cities in each tour
	void crossover();

	// AddCrossoverParentsToPopulation
	// Copy the parents back to the population
	void addCrossoverParentsToPopulation();

	// Mutation
	// Randomly swap cities in each tour in the population
	void mutation();

	// Survive
	// Pruning out _numParents cities in the population to
	// maintain the number of population
	void survive();

	// isDone
	// return true if there is no chances with in k evolution
	bool isDone();



	// DEBUG MESSAGE
	// DisplayPopulation
	void displayPopulationWithCost();

	// DisplayParents
	void displayParents();

	// Display inCities
	void displayInCities();

public:
	TSPSolver();
	// numParents % 3 == 0
	TSPSolver(int p, int sk, int numParents, int scm, char * filename);
	TSPSolver(vector<City> inputCities, bool inMeters);
	void solveIt();
	Tour getBestTour();

	void testIt();
	void displayCurBestSolution();
	void displayCurBestSolutionJustCities();
	void outputResult();
	virtual ~TSPSolver();
};
}
#endif /* TSPSOLVER_H_ */
