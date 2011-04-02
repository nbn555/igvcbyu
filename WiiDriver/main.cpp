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
#include "logging.h"
#include "YClopsConfiguration.h"
#include "YClopsModel.h"
#include "WiiController.h"

#include <mrpt/hwdrivers/CSerialPort.h>
#include "AbstractView.h"
#include "NView.h"
using namespace mrpt::utils;
using namespace std;

void signal_handler( int signum );
void handleUserInput();

ofstream of;
const string DEFAULT_POINTS_FILE = "points.txt";

/*
 * @TODO make a compass calibrator program
 * @TODO make a configuration file gui
 * @TODO make a human input hardware interface layer to remove the necessity of using the wii controller
 * @TODO create an serial port command interface to refactor the serial port sending and response sending
 */

YClopsModel * yclopsModel = NULL;
YClopsController * controller = NULL;
MVC::AbstractView * console = NULL;

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

		LOG(DEBUG4) << "Creating YClops Model" << endl;
		yclopsModel = new YClopsModel();

		LOG(DEBUG4) << "Setting up program view" << endl;
		console = new MVC::ConsoleView();
		//console = new NView(yclopsModel, NULL);

		//The wii controller constructor needs a console to write to
		//So this must be called after the console object is set up
		LOG(DEBUG4) << "Creating the User Input Device" << endl;
		//controller = new YClopsController(yclopsModel);
		controller = new WiiController(yclopsModel,console);
		console->setController(controller);

		//Set up the SIGUSR1 so we know when a button is pressed
		//this should be done after we set up the controller
		//because the signal_handler uses the controller
		LOG(DEBUG4) << "Initializing signals" << endl;
		signal(SIGINT, signal_handler);

		LOG(DEBUG4) << "Starting main loop" << endl;
		while(!controller->getIsClosing()) {
			LOG(DEBUG4) << "Iterating: " << (!controller->getIsClosing()?"true":"false") << endl;

			controller->handleEvent();

			switch(yclopsModel->ai->getCurrentState()) {
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

			if (yclopsModel->ai->getCurrentState() != mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING )
			{
				usleep(1000000/20);
				continue;
			}
			yclopsModel->ai->navigationStep();

			//If we are using a wii motor command then we need to set the speeds ourselves
			if(yclopsModel->yclops->isWiiMotorCommand()) {
				double left, right;
				controller->getRequestedSpeeds(left,right);
				yclopsModel->yclops->changeSpeeds((uint16_t)left,(uint16_t)right);
			}

		}
	} catch (...) {
		LOG(ERROR) << "Unhandled Exception caught" << endl;
		badClose = true; //We are closing with an error see the shutdown call below
	}

	of.close();

	delete controller;
	delete yclopsModel;//Need to delete the console after the model because the model destructor uses logging which requires a view
	delete console;

	return (badClose?EXIT_FAILURE:EXIT_SUCCESS);
}

void signal_handler( int signum ) {

	if( SIGINT == signum ) {
		LOG(DEBUG4) << " Signal Received " << signum << endl;
		controller->setIsClosing();
	}
}
