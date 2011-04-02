/**
 * @file YClopsController.cpp
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#include <iostream>

#include "YClopsController.h"
#include "logging.h"
//Apparently ncurses headers have to be included after any mrpt headers or it won't compile
#include <ncurses.h>
using namespace MVC;
using namespace std;

YClopsController::YClopsController(util::Observable * m):AbstractController(m), isClosing(false) {
}

YClopsController::~YClopsController() { }

bool YClopsController::getIsClosing() const {
	return this->isClosing;
}

void YClopsController::handleEvent() {

	LOG(OUT) << "In handle event" << endl;
	//if(mrpt::system::os::kbhit()) {
	//	int ch = mrpt::system::os::getch();
	int ch = wgetch(stdscr);
	LOG(DEBUG) << ch << endl;
	if(ERR != ch) {//If a button was pressed

		YClopsModel * ymodel = ((YClopsModel*)this->model);

		switch(ch){
		case 'q':
			this->isClosing = true;
			break;
		case '8':
			LOG(INFO) << "Toggling camera data" << endl;
			//yclops->toggleCameraDump();
			break;
		case '2':
			LOG(INFO) << "Toggling lidar data" << endl;
			//yclops->toggleLidarDump();
			break;
		case '4':
			LOG(INFO) << "Toggling GPS data" << endl;
			//yclops->toggleGpsDump();
			break;
		case '6':
			LOG(INFO) << "Toggling Compass data" << endl;
			//yclops->toggleCompassDump();
			break;
		case 'a':
			LOG(INFO) << "Going into Autonomous Mode" << endl;
			//yclops->setAutonomousMode();
			//ai->setChallenge(false);
			//string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
			//ai->setFileName(pointsFile,false);
			//ai->setup();
			break;
		case 'b':
			LOG(INFO) << "Going into Navigation Mode" << endl;
			//yclops->setNavigationMode();
			//ai->setChallenge(true);
			//string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
			//ai->setFileName(pointsFile,false);
			//ai->setup();
			break;
		case 'x':
			LOG(INFO) << "Idling" << endl;
			//yclops->useNullMotorCommand();
			//yclops->setIdle();
			//ai->stop();
			break;
		case 'y':
			LOG(INFO) << "Wii Motor Control" << endl;
			//yclops->useWiiMotorCommand();
			//ai->stop();
			break;
		case '1':
			LOG_WII(DEBUG4) << "Classic L1" << endl;
			break;
		case '7':
			LOG(INFO) << "Toggling Encoder data" << endl;
			//yclops->toggleEncoderDump();
			break;
		case '3':
			LOG_WII(DEBUG4) << "Classic R1" << endl;
			break;
		case '9':
			LOG_WII(DEBUG4) << "Classic R2" << endl;
			break;
		case '-':
			this->increaseLogLevel();
			break;
		case '+':
			this->decreaseLogLevel();
			break;
		case '.':
			LOG(OUT) << Log::ToString(Log::ReportingLevel()) << endl;
		default: break;
		}
/*
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
			//Beeper::beep(440,1000);
		}

		if( mbuttons & MOTE_A ) {
			LOG_WII(DEBUG4) << "Mote A" << endl;
		}

		if( mbuttons & MOTE_B ) {
			LOG_WII(DEBUG4) << "Mote B" << endl;
		}
*/
		this->model->notifyObservers();
	}

	LOG(OUT) << "Leaving handle event" << endl;
}

void YClopsController::increaseLogLevel() {
	switch(Log::ReportingLevel()) {
	case OUT:										break;
	case FATAL:										break;
	case ERROR:		Log::SetReportLevel(FATAL);		break;
	case WARNING:	Log::SetReportLevel(ERROR);		break;
	case INFO:		Log::SetReportLevel(WARNING);	break;
	case DEBUG:		Log::SetReportLevel(INFO);		break;
	case DEBUG2:	Log::SetReportLevel(DEBUG);		break;
	case DEBUG3:	Log::SetReportLevel(DEBUG2);	break;
	case DEBUG4:	Log::SetReportLevel(DEBUG3);	break;
	default:										break;
	}
	LOG(OUT) << Log::ToString(Log::ReportingLevel()) << endl;
}

void YClopsController::decreaseLogLevel() {
	switch(Log::ReportingLevel()) {
	case OUT:										break;
	case FATAL:		Log::SetReportLevel(ERROR);		break;
	case ERROR:		Log::SetReportLevel(WARNING);	break;
	case WARNING:	Log::SetReportLevel(INFO);		break;
	case INFO:		Log::SetReportLevel(DEBUG);		break;
	case DEBUG:		Log::SetReportLevel(DEBUG2);	break;
	case DEBUG2:	Log::SetReportLevel(DEBUG3);	break;
	case DEBUG3:	Log::SetReportLevel(DEBUG4);	break;
	case DEBUG4:									break;
	default:										break;
	}
	LOG(OUT) << Log::ToString(Log::ReportingLevel()) << endl;
}

