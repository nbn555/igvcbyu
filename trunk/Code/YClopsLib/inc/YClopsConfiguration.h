/**
 * @file YClopsConfiguration.h
 * @date Mar 10, 2011
 * @author tallred3
 * @brief Configuration singleton header
 */

#ifndef YCLOPSCONFIGURATION_H_
#define YCLOPSCONFIGURATION_H_

#include <mrpt/utils/CConfigFileBase.h>
#include <string>

/**
 * @brief a singleton wrapper class to the mrpt::utils::CConfigFileBase
 */
class YClopsConfiguration {
public:

	/**
	 * @brief Sets the config file path
	 * @param[in] file the config file to be read
	 */
	static void setConfigFile( const std::string );

	/**
	 * @brief initialize the config file
	 * @return CConfigFileBase reference
	 */
	static mrpt::utils::CConfigFileBase & instance();

	/**
	 * @brief removes the current config file from memory
	 */
	static void destroyConfig();
protected:
	static std::string configFileName;					//!path to the config file
	static mrpt::utils::CConfigFileBase * configFile;	//!config file pointer
	YClopsConfiguration() {};
	virtual ~YClopsConfiguration() {};

};

#endif /* YCLOPSCONFIGURATION_H_ */
