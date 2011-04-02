/**
 * @file AbstractView.cpp
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#include <cstdlib>
#include "AbstractView.h"
#include "AbstractController.h"
#include "Observable.h"
#include "logging.h"

using namespace MVC;
using namespace util;

AbstractView::AbstractView(util::Observable * m, AbstractController * c ): model(NULL), controller(NULL) {
	this->setModel(m);

	//If a controller argument is given then set the controller otherwise let it be null
	if( NULL != c ) this->setController(c);

}

AbstractView::~AbstractView() {

}

void AbstractView::setController(AbstractController * c) {
	this->controller = c;
	this->getController()->setView(this);
}

AbstractController * AbstractView::getController() {
	if(NULL == this->controller) {//If there is no controller set then get the default
		this->setController(this->defaultController(this->getModel()));
	}
	return this->controller;
}

void AbstractView::setModel( Observable * m ) {
	this->model = m;
}

Observable * AbstractView::getModel() {
	return this->model;
}

ConsoleView::ConsoleView():AbstractView(NULL) {

}

ConsoleView::~ConsoleView() {

}

void ConsoleView::log( const std::string & str ) {
			(*Log::GetOStream()) << str;

	#ifdef LOG_COLORS
			(*Log::GetOStream()) << LOG_END_COLOR;
	#endif

			Log::GetOStream()->flush();
}

void ConsoleView::update( Observable * m ) {

}
