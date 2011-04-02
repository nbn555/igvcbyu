/**
 * @file Observable.cpp
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#include <iterator>
#include <algorithm>
#include "Observable.h"

using namespace util;
using namespace std;

Observable::Observable() {

}

Observable::~Observable() {

}

bool Observable::addObserver(Observer* o) {
	this->observers.insert(this->observers.end(),o);
	return true;
}

bool Observable::removeObserver(Observer* o) {
	vector<Observer*>::iterator iter = find( this->observers.begin(), this->observers.end(), o);
	this->observers.erase(iter,iter);
	return true;
}

void Observable::notifyObservers() {
	for(vector<Observer*>::iterator iter = this->observers.begin(); iter != this->observers.end(); iter++) (*iter)->update(this);
}
