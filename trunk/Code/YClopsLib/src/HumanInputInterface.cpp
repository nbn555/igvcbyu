/**
 * @file HumanInputInterface.cpp
 * @date Mar 25, 2011
 * @author tallred3
 * @brief 
 */

#include "HumanInputInterface.h"
#include "WiiController.h"
#include "logging.h"
#include "YClopsReactiveNavInterface.h"
#include "YclopsNavigationSystem.h"
#include "logging.h"
#include "YClopsConfiguration.h"
#include "Beeper.h"

#include <iostream>

using namespace std;

LOG_LEVEL loggingLevel = DEBUG4;

extern YClopsReactiveNavInterface * yclops;
extern mrpt::reactivenav::YclopsNavigationSystem * ai;

HumanInputInterface * HumanInputInterface::interface = NULL;
bool HumanInputInterface::isClosing = false;

HumanInputInterface * HumanInputInterface::createWiiController() {

	if( NULL == HumanInputInterface::interface )
		HumanInputInterface::interface = new WiiController();
	return HumanInputInterface::interface;

}

HumanInputInterface * HumanInputInterface::instance() {
	if(NULL == HumanInputInterface::interface) {
		LOG_WII(FATAL) << "Interface Null Need to call create function" << endl;
	}
	return HumanInputInterface::interface;
}

void HumanInputInterface::destroyInterface() {
	delete HumanInputInterface::interface;
	HumanInputInterface::interface = NULL;
}


void HumanInputInterface::handleUserInput() {
	uint16_t cbuttons = 0;
	uint16_t lax = 0, lay = 0;
	uint16_t rax = 0, ray = 0;
	uint16_t la = 0;
	uint16_t ra = 0;
	uint16_t mbuttons = 0;

	HumanInputInterface * controller = HumanInputInterface::instance();

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
		yclops->setAutonomousMode();
		ai->setChallenge(false);
		string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
		ai->setFileName(pointsFile,false);
		ai->setup();
	}

	if( cbuttons & CLASSIC_B ) {
		LOG(INFO) << "Going into Navigation Mode" << endl;
		yclops->setNavigationMode();
		ai->setChallenge(true);
		string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
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
		isClosing = true;
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
	userInput = false;
}
