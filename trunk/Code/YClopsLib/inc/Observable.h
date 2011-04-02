/**
 * @file Observable.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef OBSERVABLE_H_
#define OBSERVABLE_H_

#include <vector>
#include "Observer.h"

namespace util {
	class Observable {
	public:
		Observable();
		virtual ~Observable();

		bool addObserver(Observer* o);
		bool removeObserver(Observer* o);
		void notifyObservers();
	protected:
		std::vector<Observer*> observers;
	};
}
#endif /* OBSERVABLE_H_ */
