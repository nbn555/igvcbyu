/**
 * WiiDriver for the BYU IGVC YClops submission
 * /date: Oct 21, 2010
 * /Author: Thomas Eldon Allred
 */

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <cassert>

#include "MotorController.h"
#include "MotorCommandInterface.h"
#include "Compass.h"
#include "WiiController.h"
#include "YClopsReactiveNavInterface.h"
#include "YclopsNavigationSystem.h"
#include "Beeper.h"
#include "logging.h"
#include "YClopsConfiguration.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

bool closing = false;

void signal_handler( int signum );
void shutdown( int exitStatus );
LOG_LEVEL loggingLevel = DEBUG4;

YClopsReactiveNavInterface * yclops = NULL;
ofstream of;
mrpt::reactivenav::YclopsNavigationSystem * ai = NULL;
const string DEFAULT_POINTS_FILE = "points.txt";

int main( int argc, char** argv ) {

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
		Log::SetReportLevel(loggingLevel);
		Log::SetTimeStampDisplay(false);
		Log::SetReportStreamBits(ALL_LOG&(~(WII_LOG)));
		//Log::SetReportStreamBits(ALL_LOG);
		Log::GetOStream()->precision(10);

		//Set up the SIGUSR1 so we know when a button is pressed
		LOG(DEBUG4) << "Initializing signals" << endl;
		signal(SIGUSR1, signal_handler);
		signal(SIGINT, signal_handler);

		//Initialize the WiiController
		LOG(DEBUG4) << "Creating the Wii Controller" << endl;
		WiiController::create();

		LOG(DEBUG4) << "Creating YClops Object" << endl;
		yclops = new YClopsReactiveNavInterface();

		LOG(DEBUG4) << "Creating YClops Navigation System" << endl;
		ai = new mrpt::reactivenav::YclopsNavigationSystem(*yclops, false,false);

		LOG(DEBUG4) << "loading configuration for the Navigation System" << endl;
		ai->loadConfigFile(YClopsConfiguration::instance(), YClopsConfiguration::instance());

		mrpt::poses::CPose2D curPose;
		mrpt::slam::CSimplePointsMap map;
		float curV, curW;

		LOG(DEBUG4) << "Starting main loop" << endl;
		while(!closing) {
			LOG(DEBUG4) << "Iteration again: " << (!closing?"true":"false") << endl;

			/*
			LOG(DEBUG) << "Running CurrentPose and Speeds" << endl;
			yclops->getCurrentPoseAndSpeeds(curPose, curV, curW);
			LOG(DEBUG) << "CurPose: (" << curPose.x() << "," << curPose.y() << "," << curPose.phi()*180./M_PI << ")" << endl;


			LOG(DEBUG) << "Running change speeds" << endl;
			yclops->changeSpeeds(curV, curW);
			LOG(DEBUG) << "Running senseObstacles" << endl;
			yclops->senseObstacles(map);

			//usleep(1000000/20);
			 */
			switch(ai->getCurrentState()) {
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::IDLE:
				LOG(DEBUG4) << "ai state: IDLE" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAVIGATING:
				LOG(DEBUG4) << "ai state: NAVIGATING" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::NAV_ERROR:
				LOG(DEBUG4) << "ai state: NAV_ERROR" << endl; break;
			case mrpt::reactivenav::CAbstractReactiveNavigationSystem::SUSPENDED:
				LOG(DEBUG4) << "ai state: NAV_ERROR" << endl; break;
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
		closing = false; //We are closing but we close with an error see the shutdown call below
	}

	shutdown((closing?EXIT_SUCCESS:EXIT_FAILURE));

	return EXIT_FAILURE;
}

void signal_handler( int signum ) {

	if( SIGINT == signum ) {
		LOG(DEBUG4) << " Signal Received " << signum << endl;
		if( NULL != yclops ) {
			shutdown(EXIT_FAILURE);
		}
	} else if( SIGUSR1 == signum ) {
		uint16_t cbuttons = 0;
		uint16_t lax = 0, lay = 0;
		uint16_t rax = 0, ray = 0;
		uint16_t la = 0;
		uint16_t ra = 0;
		uint16_t mbuttons = 0;

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
			ai->setChallenge(false);
			string pointsFile = YClopsConfiguration::instance().read_string("ROBOT_NAME","POINTS_FILE","points.txt");
			ai->setFileName(pointsFile,false);
			ai->setup();
		}

		if( cbuttons & CLASSIC_B ) {
			LOG(INFO) << "Going into Navigation Mode" << endl;
			yclops->setNavigationMode();
			ai->setChallenge(true);
			string pointsFile = YClopsConfiguration::instance().read_string("ROBOT_NAME","POINTS_FILE","points.txt");
			ai->setFileName(pointsFile,false);
			ai->setup();
		}

		if( cbuttons & CLASSIC_X ) {
			LOG(INFO) << "Idling" << endl;
			yclops->useNullMotorCommand();
			yclops->setIdle();
			ai->stop();
		}

		if( cbuttons & CLASSIC_Y ) {
			LOG(INFO) << "Wii Motor Control" << endl;
			yclops->useWiiMotorCommand();
			ai->stop();
		}

		if( cbuttons & CLASSIC_L1 ) {
			LOG_WII(DEBUG4) << "Classic L1" << endl;
		}

		if( cbuttons & CLASSIC_L2 ) {
			LOG(INFO) << "Toggling Encoder data" << endl;
			yclops->toggleEncoderDump();
		}

		if( cbuttons & CLASSIC_R1 ) {
			LOG_WII(DEBUG4) << "Classic R1" << endl;
		}

		if( cbuttons & CLASSIC_R2 ) {
			LOG_WII(DEBUG4) << "Classic R2" << endl;
		}

		if( cbuttons & CLASSIC_SELECT ) {
			switch(loggingLevel) {
			case DISABLE:							break;
			case FATAL:		loggingLevel = ERROR; 	break;
			case ERROR:		loggingLevel = WARNING;	break;
			case WARNING:	loggingLevel = INFO;	break;
			case INFO:		loggingLevel = DEBUG;	break;
			case DEBUG:		loggingLevel = DEBUG2;	break;
			case DEBUG2:	loggingLevel = DEBUG3;	break;
			case DEBUG3:	loggingLevel = DEBUG4;	break;
			case DEBUG4:							break;
			default:								break;
			}
			(*Log::GetOStream()) << "Log level: " << Log::ToString( loggingLevel ) << endl;
			Log::SetReportLevel(loggingLevel);
		}

		if( cbuttons & CLASSIC_HOME ) {
			LOG(DEBUG4) << "Shutdown button pressed" << endl;
			closing = true;
		}

		if( cbuttons & CLASSIC_START ) {
			switch(loggingLevel) {
			case DISABLE:							break;
			case FATAL: 						 	break;
			case ERROR:		loggingLevel = FATAL;	break;
			case WARNING:	loggingLevel = ERROR;	break;
			case INFO:		loggingLevel = WARNING;	break;
			case DEBUG:		loggingLevel = INFO;	break;
			case DEBUG2:	loggingLevel = DEBUG;	break;
			case DEBUG3:	loggingLevel = DEBUG2;	break;
			case DEBUG4:	loggingLevel = DEBUG3;	break;
			default:								break;
			}
			(*Log::GetOStream()) << "Log level: " << Log::ToString( loggingLevel ) << endl;
			Log::SetReportLevel(loggingLevel);

		}

		if( mbuttons & MOTE_D_UP ) {
			LOG_WII(DEBUG4) << "Mote D Up" << endl;
		}

		if( mbuttons & MOTE_D_LEFT ) {
			LOG_WII(DEBUG4) << "Mote D Left" << endl;
		}

		if( mbuttons & MOTE_D_RIGHT ) {
			LOG_WII(DEBUG4) << "Mote D Right" << endl;
		}

		if( mbuttons & MOTE_D_DOWN ) {
			LOG_WII(DEBUG4) << "Mote D Down" << endl;
		}

		if( mbuttons & MOTE_1 ) {
			LOG_WII(DEBUG4) << "Mote 1" << endl;
		}

		if( mbuttons & MOTE_2 ) {
			LOG_WII(DEBUG4) << "Mote 2" << endl;
		}

		if( mbuttons & MOTE_PLUS ) {
			LOG_WII(DEBUG4) << "Mote Plus" << endl;
		}

		if( mbuttons & MOTE_MINUS ) {
			LOG_WII(DEBUG4) << "Mote Minus" << endl;
		}

		if( mbuttons & MOTE_HOME ) {
			LOG(DEBUG4) << "Playing beep" << endl;
			Beeper::beep(440,1000);
		}

		if( mbuttons & MOTE_A ) {
			LOG_WII(DEBUG4) << "Mote A" << endl;
		}

		if( mbuttons & MOTE_B ) {
			LOG_WII(DEBUG4) << "Mote B" << endl;
		}
	}
}

void shutdown( int exitStatus ) {
	LOG(INFO) << "Shutting down YClops" << endl;
	WiiController::destroyReference();

	//sleep(1);

	delete yclops;
	yclops = NULL;
//	delete ai;
//	ai = NULL;

	//sleep(1);

	exit(exitStatus);
}
