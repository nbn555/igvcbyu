/**
 * @file YClopsModel.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef YCLOPSMODEL_H_
#define YCLOPSMODEL_H_

#include "YClopsReactiveNavInterface.h"
#include "YclopsNavigationSystem.h"

class YClopsModel {
public:
	YClopsModel();
	virtual ~YClopsModel();

	YClopsReactiveNavInterface * yclops;
	mrpt::reactivenav::YclopsNavigationSystem * ai;

};

#endif /* YCLOPSMODEL_H_ */
