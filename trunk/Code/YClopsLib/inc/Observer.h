/**
 * @file Observer.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

namespace util {
	class Observable;
	class Observer {
	public:
		Observer();
		virtual ~Observer();
		virtual void update( Observable * m ) = 0;
	};
}

#endif /* OBSERVER_H_ */
