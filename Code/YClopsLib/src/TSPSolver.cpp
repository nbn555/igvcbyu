/*
 * TSPSolver.cpp
 *
 *  Created on: Mar 1, 2011
 *      Author: Tobias
 */

#include "TSPSolver.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <algorithm>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
using namespace std;
namespace lkinhou {

bool byCost(const Tour &a, const Tour &b) {
	return (a._cost < b._cost);
}

bool byCostReverse(const Tour &a, const Tour &b) {
	return (a._cost > b._cost);
}

TSPSolver::TSPSolver() {
	cout << "DO NOT CALL THIS!" << endl;
}

TSPSolver::TSPSolver(int population, int stopK, int numParents, int scm, char * filename) {

	// initialize random seed
	srand((unsigned) time(0));

	// init to a huge number
	_curBestTour._cost = 10000000;

	_changeCount = 0;

	_numPopulation = population;
	_stopK = stopK;
	assert(numParents >= 3 && numParents % 3 == 0);
	_numParents = numParents;
	assert(scm >= 2);
	_numSwapCityMutation = scm;

	ifstream inFile(filename);
	if (inFile.is_open()) {
		int i = 0;
		while (inFile.good()) {
			City tmp;
			tmp.id = i;
			inFile >> tmp.x >> tmp.y;
			inCities.push_back(tmp);
			i++;
		}
		inFile.close();
		_numCity = inCities.size();
		assert(_numCity >= 10);
	} else {
		cout << "Unable to open file";
		return;
	}

	if (DEBUG)
		displayInCities();
}

TSPSolver::TSPSolver(vector<City> inputCities, bool inMeters){
	// initialize random seed
	srand((unsigned) time(0));

	// init to a huge number
	_curBestTour._cost = 10000000;

	_changeCount = 0;

	int numPopulation = 10;
	int numStop = 50000;
	int numParents = 6;
	int numSwapCitiesInMutation = 4;

	_numPopulation = numPopulation;
	_stopK = numStop;
	assert(numParents >= 3 && numParents % 3 == 0);
	_numParents = numParents;
	assert(numSwapCitiesInMutation >= 2);
	_numSwapCityMutation = numSwapCitiesInMutation;

	_numCity = inputCities.size();
	assert(_numCity >= 10);
	for(int i=0; i < _numCity; i++){
		City tmp;
		tmp.x = inputCities[i].x;
		tmp.y = inputCities[i].y;
		inCities.push_back(tmp);
	}
}

// GetRandomInteger
// lr: left range
// rr: right range
// return an random integer in [lr,rr]
int TSPSolver::getRandomInteger(int lr, int rr) {
	assert(lr < rr);
	return rand() % (rr - lr) + lr;
}

// Initialization
// Randomly generate _numPopulation valid routes
void TSPSolver::initialization() {
	if (DEBUG)
		cout << "initialization" << endl;
	int * checkTable = new int[_numCity];
	int addCount = 0;
	// init checkTable to zeros
	for (int i = 0; i < _numCity; i++) {
		checkTable[i] = 0;
	}

	for (int i = 0; i < _numPopulation; i++) {
		Tour tour;
		while (addCount != _numCity) {
			int r = getRandomInteger(0, _numCity);
			if (checkTable[r] == 0) {
				addCount++;
				checkTable[r] = 1;
				tour.addCity(r);
			}
		}
		_population.push_back(tour);
		addCount = 0;
		// init checkTable to zeros
		for (int i = 0; i < _numCity; i++) {
			checkTable[i] = 0;
		}
	}
}

// GetDist
// a: a city
// b: another city
// return the display between these two cities
double TSPSolver::getDist(const City & a, const City & b) {
	if(_inMeters == true)
		return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
	else{
		cout << "TSPSolver::getDist:   DOESN't SUPPORT NON-METER UNIT" << endl;
		return -1;
	}
}

// Evaluation
// Calculating the cost of each tour in the population
// And save the best tour so far
void TSPSolver::evaluation() {
	if (DEBUG)
		cout << "evaluation" << endl;
	double sumDist = 0;
	int j;
	for (int i = 0; i < _numPopulation; i++) {
		for (j = 0; j < _numCity - 1; j++) {
			int cida = _population[i].getCity(j);
			int cidb = _population[i].getCity(j + 1);
			sumDist += getDist(inCities[cida], inCities[cidb]);
		}
		int cida = _population[i].getCity(j);
		int cidb = _population[i].getCity(0);
		sumDist += getDist(inCities[cida], inCities[cidb]);
		_population[i]._cost = sumDist;
		sumDist = 0;
	}
}

// ParentSelection
// Select _numParents parents in the population
// note: _numParents % 3 == 0
// parents: 1/3 min cost , 1/3 mid cost, 1/3 high cost from the population
void TSPSolver::parentSelection() {
	if (DEBUG)
		cout << "parentSelection" << endl;
	double totalSumDist = 0;
	for (int i = 0; i < _population.size(); i++) {
		totalSumDist += _population[i]._cost;
	}

	sort(_population.begin(), _population.end(), byCost);

	// update the current data if necessary
	if (_population[0]._cost < _curBestTour._cost) {
		_curBestTour.copy(_population[0]);
		_curBestTour._cost = _population[0]._cost;
		_changeCount = 0;
	} else {
		_changeCount++;
	}

	int numPart = _numParents / 3;
	for (int i = 0; i < numPart; i++) {
		Tour tourHigh;
		Tour tourMid;
		Tour tourLow;
		tourHigh.copy(_population[_population.size() - 1 - i]);
		tourMid.copy(_population[_population.size() / 2 - 1 + i]);
		tourLow.copy(_population[i]);
		_parents.push_back(tourHigh);
		_parents.push_back(tourMid);
		_parents.push_back(tourLow);
	}
}

// Crossover
// Combine parents by swapping the cities in each tour
// _numCity >= 10, so we divide a tour into 3 parts
// and swap the 1st part with the 3rd part
void TSPSolver::crossover() {
	if (DEBUG)
		cout << "crossover" << endl;
	int part = _numCity / 3;
	for (int p = 0; p < _numParents; p++) {
		for (int i = 0; i < part; i++) {
			int a = _parents[p].getCity(i);
			int b = _parents[p].getCity(_numCity - 1 - i);
			_parents[p].setCity(i, b);
			_parents[p].setCity(_numCity - 1 - i, a);
		}
	}
}

// AddCrossoverParentsToPopulation
// Copy the parents back to the population
void TSPSolver::addCrossoverParentsToPopulation() {
	for (int i = 0; i < _parents.size(); i++) {
		_population.push_back(_parents[i]);
	}
	_parents.clear();
}

// Mutation
// Randomly swap _numSwapCityMutation cities in each tour in the population
// :::: It doesn't matter if r is randomly generating twice or many times.
void TSPSolver::mutation() {
	if (DEBUG)
		cout << "mutation" << endl;
	for (int p = 0; p < _numPopulation; p++) {
		for (int j = 0; j < _numSwapCityMutation; j++) {
			int r = getRandomInteger(0, _numCity);
			//cout << "Swap " << r << " with " << _numCity - 1 - j << endl;
			int a = _population[p].getCity(r);
			int b = _population[p].getCity(_numCity - 1 - j);
			_population[p].setCity(r, b);
			_population[p].setCity(_numCity - 1 - j, a);
		}
	}
}

// Survive
// Pruning out _numParents cities in the population to
// maintain the number of population
// :::: drop _numParents the highest cost
void TSPSolver::survive() {
	if (DEBUG)
		cout << "surive" << endl;
	sort(_population.begin(), _population.end(), byCost);
	for (int i = 0; i < _numParents; i++) {
		_population.pop_back();
	}
}

// isDone
// return true if there is no chances with in _stopK evolution
bool TSPSolver::isDone() {
	if (_changeCount < _stopK)
		return false;
	else
		return true;
}

void TSPSolver::solveIt() {
	int t = 0;
	initialization();
	evaluation();
	while (!isDone()) {
		parentSelection();
		crossover();
		addCrossoverParentsToPopulation();
		mutation();
		evaluation();
		survive();
		t++;
	}
	if (DEBUG) {
		cout << "Done: " << endl;
		displayCurBestSolution();
	} else {
		displayCurBestSolutionJustCities();
	}
}

Tour TSPSolver::getBestTour(){
	Tour t;
	t.copy(_curBestTour);
	return t;
}

void TSPSolver::testIt() {
	cout << "Testing it" << endl;
	initialization();
	evaluation();
	if (DEBUG)
		displayPopulationWithCost();
	parentSelection();
	if (DEBUG) {
		displayPopulationWithCost();
		displayParents();
	}
	crossover();
	if (DEBUG) {
		displayParents();
	}
	addCrossoverParentsToPopulation();
	if (DEBUG) {
		displayPopulationWithCost();
	}
	mutation();
	if (DEBUG) {
		displayPopulationWithCost();
	}
	evaluation();
	if (DEBUG) {
		displayPopulationWithCost();
	}
	if (DEBUG) {
		displayCurBestSolution();
	}

	survive();
	if (DEBUG) {
		displayPopulationWithCost();
	}
}

void TSPSolver::displayCurBestSolution() {
	cout << "==========================" << endl;
	cout << "displayCurBestSolution" << endl;
	cout << "==========================" << endl;
	cout << "cost: " << _curBestTour._cost << endl;
	for (int i = 0; i < _curBestTour.getTourSize(); i++) {
		cout << _curBestTour.getCity(i) << " ";
	}
	cout << _curBestTour.getCity(0);
	cout << endl << "Detail: " << endl;
	for (int i = 0; i < _curBestTour.getTourSize(); i++) {
		cout << inCities[_curBestTour.getCity(i)].x << " "
				<< inCities[_curBestTour.getCity(i)].y << endl;
	}
	cout << inCities[_curBestTour.getCity(0)].x << " "
			<< inCities[_curBestTour.getCity(0)].y << endl;
}

void TSPSolver::displayCurBestSolutionJustCities() {
	for (int i = 0; i < _curBestTour.getTourSize(); i++) {
		cout << inCities[_curBestTour.getCity(i)].x << " "
				<< inCities[_curBestTour.getCity(i)].y << endl;
	}
	cout << inCities[_curBestTour.getCity(0)].x << " "
			<< inCities[_curBestTour.getCity(0)].y << endl;
}

// DEBUG MESSAGE
// DisplayPopulation
void TSPSolver::displayPopulationWithCost() {
	cout << "==========================" << endl;
	cout << "displayPopulationWithCost" << endl;
	cout << "==========================" << endl;
	for (int i = 0; i < _population.size(); i++) {
		cout << "p" << i << ": ";
		for (int j = 0; j < _population[0].getTourSize(); j++) {
			cout << _population[i].getCity(j) << " ";
		}
		cout << " | " << _population[i]._cost;
		cout << endl;
	}
}

// DisplayParents
void TSPSolver::displayParents() {
	cout << "==========================" << endl;
	cout << "displayParents" << endl;
	cout << "==========================" << endl;
	for (int i = 0; i < _parents.size(); i++) {
		cout << "p" << i << ": ";
		for (int j = 0; j < _parents[0].getTourSize(); j++) {
			cout << _parents[i].getCity(j) << " ";
		}
		cout << " | " << _parents[i]._cost;
		cout << endl;
	}
}

// DisplayInCities
void TSPSolver::displayInCities() {
	for (int i = 0; i < inCities.size(); i++) {
		cout << "id: " << inCities[i].id << " x: " << inCities[i].x << " y: "
				<< inCities[i].y << endl;
	}
}

TSPSolver::~TSPSolver() {
	return;
}

}
