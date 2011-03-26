/**
 * @brief WiiDriver for the BYU IGVC YClops submission
 * /date Oct 21, 2010
 * /Author Thomas Eldon Allred
 */

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <cassert>

#include "MotorController.h"
#include "MotorCommandInterface.h"
#include "Compass.h"
#include "HumanInputInterface.h"
#include "WiiController.h"
#include "YClopsReactiveNavInterface.h"
#include "YclopsNavigationSystem.h"
#include "logging.h"
#include "YClopsConfiguration.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

void signal_handler( int signum );
void shutdown( int exitStatus );
void handleUserInput();

YClopsReactiveNavInterface * yclops = NULL;
mrpt::reactivenav::YclopsNavigationSystem * ai = NULL;
ofstream of;
const string DEFAULT_POINTS_FILE = "points.txt";

/*
 * @TODO make a compass calibrator program
 * @TODO make a configuration file gui
 * @TODO make a human input hardware interface layer to remove the necessity of using the wii controller
 * @TODO create an serial port command interface to refactor the serial port sending and response sending
 */
int main( int argc, char** argv ) {

	bool badClose = false;
	try{
		//test if the config file is passed in
		if(argc < 2) {
			//if not print exit message and die
			cout << "Usage: " << argv[0] << " file.ini" << endl;
			exit(EXIT_FAILURE);
		}else {
			//if so load the config file
			YClopsConfiguration::setConfigFile(string(argv[1]));
		}

		of.open("dump.txt");

		//Set up logging
		//Log::SetLogFile(&of);
		Log::SetLogFile(&cout);
		Log::SetReportLevel(DEBUG4);
		Log::SetTimeStampDisplay(false);
		Log::SetReportStreamBits(ALL_LOG&(~(WII_LOG)));
		//Log::SetReportStreamBits(ALL_LOG);
		Log::GetOStream()->precision(10);

		//Set up the SIGUSR1 so we know when a button is pressed
		LOG(DEBUG4) << "Initializing signals" << endl;
		signal(SIGUSR1, signal_handler);
		signal(SIGINT, signal_handler);

		//Initialize the WiiController
		LOG(DEBUG4) << "Creating the User Input Device" << endl;
		HumanInputInterface::createWiiController();

		LOG(DEBUG4) << "Creating YClops Object" << endl;
		yclops = new YClopsReactiveNavInterface();

		LOG(DEBUG4) << "Creating YClops Navigation System" << endl;
		ai = new mrpt::reactivenav::YclopsNavigationSystem(*yclops, false,false);

		LOG(DEBUG4) << "loading configuration for the Navigation System" << endl;
		ai->loadConfigFile(YClopsConfiguration::instance(), YClopsConfiguration::instance());

		LOG(DEBUG4) << "Starting main loop" << endl;
		while(!HumanInputInterface::getClosingFlag()) {
			LOG(DEBUG4) << "Iterating: " << (!HumanInputInterface::getClosingFlag()?"true":"false") << endl;

			if(HumanInputInterface::instance()->getInputFlag())
				HumanInputInterface::instance()->handleUserInput();

			switch(ai->getCurrentState()) {
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::IDLE:
				LOG(DEBUG4) << "ai state: IDLE" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING:
				LOG(DEBUG4) << "ai state: NAVIGATING" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAV_ERROR:
				LOG(DEBUG4) << "ai state: NAV_ERROR" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::SUSPENDED:
				LOG(DEBUG4) << "ai state: SUSPENDED" << endl; break;
			default:
				break;
			}

			if (ai->getCurrentState() != mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING )
			{
				usleep(1000000/20);
				continue;
			}
			ai->navigationStep();

		}
	} catch (...) {
		LOG(ERROR) << "Unhandled Exception caught" << endl;
		badClose = true; //We are closing with an error see the shutdown call below
	}

	shutdown((badClose?EXIT_FAILURE:EXIT_SUCCESS));

	return EXIT_FAILURE;
}

void signal_handler( int signum ) {

	if( SIGINT == signum ) {
		LOG(DEBUG4) << " Signal Received " << signum << endl;
		if( NULL != yclops ) {
			shutdown(EXIT_FAILURE);
		}
	} else if( SIGUSR1 == signum ) {
		HumanInputInterface::instance()->setInputFlag();
	}
}

void shutdown( int exitStatus ) {
	LOG(INFO) << "Shutting down YClops" << endl;

	LOG(INFO) << "Closing Input Interface" << endl;
	HumanInputInterface::destroyInterface();

	//ai assumes yclops is still valid so it can't be deleted before ai
	LOG(INFO) << "Deleting ai" << endl;
	delete ai;	ai = NULL;

	LOG(INFO) << "Deleting yclops" << endl;
	delete yclops;	yclops = NULL;

	exit(exitStatus);
}
