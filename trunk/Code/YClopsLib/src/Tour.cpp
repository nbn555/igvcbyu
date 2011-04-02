/*
 * Tour.cpp
 *
 *  Created on: Mar 1, 2011
 *      Author: Tobias
 */

#include "Tour.h"
#include <vector>
using namespace std;
namespace lkinhou {
Tour::Tour() {
	// TODO Auto-generated constructor stub

}

Tour::~Tour() {
	// TODO Auto-generated destructor stub
}

int Tour::getCity(int j) {
	return _tourCities[j];
}
void Tour::addCity(int i) {
	_tourCities.push_back(i);
}

void Tour::setCity(int pos, int val) {
	_tourCities[pos] = val;
}

int Tour::getTourSize() {
	return _tourCities.size();
}

void Tour::copy(const Tour & other) {
	_tourCities.clear();
	for (int i = 0; i < other._tourCities.size(); i++) {
		_tourCities.push_back(other._tourCities[i]);
		_cost = other._cost;
	}
}
}
