/**
 * @file AbstractController.cpp
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#include "AbstractController.h"

using namespace MVC;
using namespace util;

AbstractController::AbstractController(Observable * m) {
	this->setModel(m);
}

AbstractController::~AbstractController() {

}

void AbstractController::setView( AbstractView * v ) {
	this->view = v;
}

AbstractView * AbstractController::getView() {
	return this->view;
}

void AbstractController::setModel( Observable * m ) {
	this->model = m;
}

Observable * AbstractController::getModel() {
	return this->model;
}
