/**
 * @file AbstractView.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef ABSTRACTVIEW_H_
#define ABSTRACTVIEW_H_

#include <cstdlib>
#include <sstream>

#include "Observer.h"
#include "Observable.h"

namespace MVC {
	class AbstractController;
	//class ViewUpdate;

	class AbstractView: public util::Observer {
	public:
		AbstractView(util::Observable * m, AbstractController * c = NULL);
		virtual ~AbstractView();

		void setController(AbstractController * c);
		AbstractController * getController();

		void setModel( util::Observable * m );
		util::Observable * getModel();

		virtual AbstractController * defaultController(util::Observable * m) { return NULL; };

		virtual void log( const std::string & str ) = 0;

	protected:
		util::Observable * model;
		AbstractController * controller;
	};

	class ConsoleView: public AbstractView {
	public:
		ConsoleView();
		virtual ~ConsoleView();
		void log( const std::string & str );
		void update( util::Observable * m );
	};
}

#endif /* ABSTRACTVIEW_H_ */
