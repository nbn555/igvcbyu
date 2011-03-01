/**
 * WiiDriver for the BYU IGVC YClops submission
 * /date: Oct 21, 2010
 * /Author: Thomas Eldon Allred
 */

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <cassert>

#include <mrpt/utils/CConfigFile.h>

#include "MotorController.h"
#include "MotorCommandInterface.h"
#include "Compass.h"
#include "WiiController.h"
#include "YClopsReactiveNavInterface.h"
#include "Beeper.h"
#include "logging.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

void signal_handler( int signum );
LOG_LEVEL loggingLevel = DEBUG4;

YClopsReactiveNavInterface * yclops = NULL;

int main( int argc, char** argv ) {

	try{
		//create the config file
		CConfigFile configFile;

		//test if the config file is passed in
		if(argc < 2) {
			//if not print exit message and die
			cout << "Usage: " << argv[0] << " file.ini" << endl;
			exit(EXIT_FAILURE);
		}else {
			//if so load the config file
			configFile.setFileName(string(argv[1]));
		}

		//Set up logging
		Log::SetLogFile(&cout);
		Log::SetReportLevel(loggingLevel);

		//Set up the SIGUSR1 so we know when a button is pressed
		signal(SIGUSR1, signal_handler);
		signal(SIGINT, signal_handler);

		//Initialize the WiiController
		WiiController::create();

		//Set the motor controller to connect to the port name in the config file
		MotorController::setConfigFile( (mrpt::utils::CConfigFileBase*)(&configFile) );

		yclops = new YClopsReactiveNavInterface( configFile );

		yclops->useNullMotorCommand();

		while(1) {
			mrpt::poses::CPose2D curPose;
			mrpt::slam::CSimplePointsMap map;
			float curV, curW;

			yclops->getCurrentPoseAndSpeeds(curPose, curV, curW);
			yclops->changeSpeeds(curV, curW);
			yclops->senseObstacles(map);

		}
	} catch (...) {
		delete yclops;
	}

	return 0;
}

void signal_handler( int signum ) {

	if( SIGINT == signum ) {
		LOG(DEBUG4) << " Signal Received " << signum << endl;
		if( NULL != yclops ) {
			delete yclops;
			yclops = NULL;
			exit(EXIT_FAILURE);
		}
	} else if( SIGUSR1 == signum ) {
		uint16_t cbuttons;
		uint16_t lax, lay;
		uint16_t rax, ray;
		uint16_t la;
		uint16_t ra;
		uint16_t mbuttons;

		WiiController * controller = WiiController::getReference();

		controller->getClassicButtons(cbuttons);
		controller->getLeftStick(lax,lay);
		controller->getRightStick(rax,ray);
		controller->getLeftAnalog(la);
		controller->getRightAnalog(ra);
		controller->getMoteButtons(mbuttons);

		if( cbuttons & CLASSIC_D_UP ) {
			LOG(INFO) << "Toggling camera data" << endl;
			yclops->toggleCameraDump();
		}

		if( cbuttons & CLASSIC_D_DOWN ) {
			LOG(INFO) << "Toggling lidar data" << endl;
			yclops->toggleLidarDump();
		}

		if( cbuttons & CLASSIC_D_LEFT ) {
			LOG(INFO) << "Toggling GPS data" << endl;
			yclops->toggleGpsDump();
		}

		if( cbuttons & CLASSIC_D_RIGHT ) {
			LOG(INFO) << "Toggling Compass data" << endl;
			yclops->toggleCompassDump();
		}

		if( cbuttons & CLASSIC_A ) {
			LOG(INFO) << "Going into Autonomous Mode" << endl;
			yclops->setAutonomusMode();
		}

		if( cbuttons & CLASSIC_B ) {
			LOG(INFO) << "Going into Navigation Mode" << endl;
			yclops->setNavigationMode();
		}

		if( cbuttons & CLASSIC_X ) {
			LOG(INFO) << "Idling" << endl;
			LOG(DEBUG4) << "YClops: " << yclops << endl;

			yclops->useNullMotorCommand();
			yclops->setIdle();
		}

		if( cbuttons & CLASSIC_Y ) {
			LOG(INFO) << "Wii Motor Control" << endl;
			yclops->useWiiMotorCommand();
		}

		if( cbuttons & CLASSIC_L1 ) {
			LOG(DEBUG4) << "Classic L1" << endl;
		}

		if( cbuttons & CLASSIC_L2 ) {
			LOG(DEBUG4) << "Classic L2" << endl;
		}

		if( cbuttons & CLASSIC_R1 ) {
			LOG(DEBUG4) << "Classic R1" << endl;
		}

		if( cbuttons & CLASSIC_R2 ) {
			LOG(DEBUG4) << "Classic R2" << endl;
		}

		if( cbuttons & CLASSIC_SELECT ) {
			LOG(DEBUG4) << "Classic Select" << endl;
			switch(loggingLevel) {
			case DISABLE:							break;
			case FATAL: 	loggingLevel = ERROR; 	break;
			case ERROR: 	loggingLevel = WARNING;	break;
			case WARNING:	loggingLevel = INFO;	break;
			case INFO:		loggingLevel = DEBUG;	break;
			case DEBUG:		loggingLevel = DEBUG2;	break;
			case DEBUG2:	loggingLevel = DEBUG3;	break;
			case DEBUG3:	loggingLevel = DEBUG4;	break;
			case DEBUG4:							break;
			default:								break;
			}
			Log::SetReportLevel(loggingLevel);
		}

		if( cbuttons & CLASSIC_HOME ) {
			delete yclops;
			yclops = NULL;
			exit(0);
			//LOG(DEBUG4) << "Playing beep" << endl;
			//Beeper::beep(440,1000);
		}

		if( cbuttons & CLASSIC_START ) {
			LOG(DEBUG4) << "Classic Start" << endl;
			switch(loggingLevel) {
			case DISABLE:							break;
			case FATAL: 						 	break;
			case ERROR: 	loggingLevel = FATAL;	break;
			case WARNING:	loggingLevel = ERROR;	break;
			case INFO:		loggingLevel = WARNING;	break;
			case DEBUG:		loggingLevel = INFO;	break;
			case DEBUG2:	loggingLevel = DEBUG;	break;
			case DEBUG3:	loggingLevel = DEBUG2;	break;
			case DEBUG4:	loggingLevel = DEBUG3;	break;
			default:								break;
			}
			Log::SetReportLevel(loggingLevel);

		}

		if( mbuttons & MOTE_D_UP ) {

		}

		if( mbuttons & MOTE_D_LEFT ) {

		}

		if( mbuttons & MOTE_D_RIGHT ) {

		}

		if( mbuttons & MOTE_D_DOWN ) {

		}

		if( mbuttons & MOTE_1 ) {

		}

		if( mbuttons & MOTE_2 ) {

		}

		if( mbuttons & MOTE_PLUS ) {

		}

		if( mbuttons & MOTE_MINUS ) {

		}

		if( mbuttons & MOTE_HOME ) {

		}

		if( mbuttons & MOTE_A ) {

		}

		if( mbuttons & MOTE_B ) {

		}
#ifdef DEBUG
		LOG(DEBUG4) << "classic buttons ";
		for( uint16_t bitMask = (1 << (sizeof(bitMask)*8-1)); bitMask; bitMask >>= 1) {
			if( bitMask & cbuttons ) {
				LOG(DEBUG4) << '1';
			} else {
				LOG(DEBUG4) << '0';
			}
		}

		cout << "Classic l analog stick ";
		cout << lax << "," << lay << endl;

		cout << "Classic r analog stick ";
		cout << rax << "," << ray << endl;

		cout << "Classic analog l ";
		cout << la << endl;

		cout << "Classic analog r ";
		cout << ra << endl;

		cout << "Mote buttons ";
		for( uint16_t bitMask = (1<<(sizeof(bitMask)*8-1));bitMask;bitMask>>=1){
			if(bitMask & mbuttons) {
				cout << '1';
			} else {
				cout << '0';
			}
		}
		cout << endl;
#endif
	}
}
