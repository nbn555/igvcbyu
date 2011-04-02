/**
 * @file AbstractController.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef ABSTRACTCONTROLLER_H_
#define ABSTRACTCONTROLLER_H_

#include "AbstractView.h"
#include "Observable.h"

namespace MVC {

	class AbstractController {
	public:
		AbstractController(util::Observable * m);
		virtual ~AbstractController();

		void setView( AbstractView * v );
		AbstractView * getView();

		void setModel( util::Observable * m );
		util::Observable * getModel();

	protected:
		util::Observable * model;
		AbstractView * view;
	};
}
#endif /* ABSTRACTCONTROLLER_H_ */
