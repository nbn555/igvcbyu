/*
 * Tour.h
 *
 *  Created on: Mar 1, 2011
 *      Author: Tobias
 */

#ifndef TOUR_H_
#define TOUR_H_
#include<vector>
using namespace std;
namespace lkinhou {
class Tour {
private:
	vector<int> _tourCities;
public:
	double _cost;
	Tour();
	virtual ~Tour();
	int getCity(int j);
	void addCity(int i);
	void setCity(int pos, int val);
	int getTourSize();
	void copy(const Tour & other);
};
}
#endif /* TOUR_H_ */
