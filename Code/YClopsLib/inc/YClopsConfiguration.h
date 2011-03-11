/*
 * YClopsConfiguration.h
 *
 *  Created on: Mar 10, 2011
 *      Author: tallred3
 */

#ifndef YCLOPSCONFIGURATION_H_
#define YCLOPSCONFIGURATION_H_

#include <mrpt/utils/CConfigFileBase.h>
#include <string>

class YClopsConfiguration {
public:
	virtual ~YClopsConfiguration() {};
	static void setConfigFile( const std::string );
	static mrpt::utils::CConfigFileBase & instance();
	static void destroyConfig();
protected:
	static std::string configFileName;
	static mrpt::utils::CConfigFileBase * configFile;
	YClopsConfiguration() {};
};

#endif /* YCLOPSCONFIGURATION_H_ */
