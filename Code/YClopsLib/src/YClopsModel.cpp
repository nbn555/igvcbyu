/**
 * @file YClopsModel.cpp
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#include "YClopsModel.h"
#include "logging.h"
#include "YClopsConfiguration.h"

using namespace std;

YClopsModel::YClopsModel():yclops(new YClopsReactiveNavInterface()),
ai(new mrpt::reactivenav::YclopsNavigationSystem(*yclops, false,false)) {

	LOG(DEBUG4) << "loading configuration for the Navigation System" << endl;
	ai->loadConfigFile(YClopsConfiguration::instance(), YClopsConfiguration::instance());
}

YClopsModel::~YClopsModel() {
	LOG(INFO) << "Shutting down YClops" << endl;

	//ai assumes yclops is still valid so it can't be deleted before ai
	LOG(INFO) << "Deleting ai" << endl;
	delete ai;	ai = NULL;

	LOG(INFO) << "Deleting yclops" << endl;
	delete yclops;	yclops = NULL;

}
