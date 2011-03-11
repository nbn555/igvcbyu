/*
 * YClopsConfiguration.cpp
 *
 *  Created on: Mar 10, 2011
 *      Author: tallred3
 */

#include "YClopsConfiguration.h"

#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

mrpt::utils::CConfigFileBase * YClopsConfiguration::configFile = NULL;
std::string YClopsConfiguration::configFileName = "";

void YClopsConfiguration::setConfigFile( const std::string fileName ) {
	YClopsConfiguration::configFileName = fileName;
}

CConfigFileBase & YClopsConfiguration::instance() {
	if( NULL == YClopsConfiguration::configFile ) {
		YClopsConfiguration::configFile = new CConfigFile(YClopsConfiguration::configFileName);
	}
	return *(YClopsConfiguration::configFile);
}

void YClopsConfiguration::destroyConfig() {
	delete YClopsConfiguration::configFile;
	YClopsConfiguration::configFile = NULL;
}
