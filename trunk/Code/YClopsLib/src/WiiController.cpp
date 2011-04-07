/**
 * @file WiiController.cpp
 * @date Mar 29, 2011
 * @author tallred3
 * @brief 
 */

#include <cwiid.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <string>
#include <sstream>

#include "WiiController.h"
#include "YClopsConfiguration.h"
#include "logging.h"

cwiid_mesg_callback_t cwiid_callback;

using namespace util;
using namespace std;

static bool toCopy = false;
static uint16_t mButtonsPressed;	//!<The true for which buttons were just pressed
static uint16_t mButtonsReleased;	//!<True for which buttons were just released
static uint16_t mButtons;			//!<true for which buttons are pressed

static uint16_t cButtonsPressed;	//!<True for which buttons were just pressed
static uint16_t cButtonsReleased;	//!<True for which buttons were just released
static uint16_t cButtons;			//!<True for the state of the pressed buttons

static uint16_t lXa, lYa;			//!<classic controller left stick state
static uint16_t rXa, rYa;			//!<classic controller right stick state
static uint16_t lA, rA;				//!<classic controller analog state


WiiController::WiiController(util::Observable * m, MVC::AbstractView * v):YClopsController(m) {
	this->setView(v);
	bdaddr_t bdaddr = *BDADDR_ANY;	//!Get the most unspecific bluetooth address

	this->wiiMote = NULL;

	int dev_id = -1;
	bool isBluetoothFound = false;
	while(!isBluetoothFound) {
		LOG_WII(DEBUG2) << "Searching for Bluetooth interface" << endl;
		if((dev_id = hci_get_route(NULL)) == -1) {		//!Scan for bluetooth devices
			LOG_WII(ERROR) << "Couldn't find Bluetooth Interface" << endl;
		} else isBluetoothFound = true;					//!Once we find one we assume it's a wii mote
	}

	LOG_WII(DEBUG2) << "Bluetooth found" << endl;

	LOG(OUT) << "Press buttons 1 + 2 to put WiiMote in Discoverable mode" << endl;

	while(-1 ==  cwiid_find_wiimote(&bdaddr, 5)) {}

	while(!this->wiiMote) {//Try to open the wii controller
		this->wiiMote = cwiid_open(&bdaddr,CWIID_FLAG_MESG_IFC);
		if(!this->wiiMote) {
			LOG_WII(ERROR) << "Connect Fail Retrying" << endl;
		}
	}

	LOG_WII(DEBUG2) << "Connected WiiMote" << endl;

	cwiid_set_mesg_callback(this->wiiMote, &cwiid_callback);  //Set up the message call back handler
	cwiid_command(this->wiiMote, CWIID_CMD_RPT_MODE, CWIID_RPT_BTN | CWIID_RPT_CLASSIC ); //Set up the messages we want reported

	LOG_WII(DEBUG2) << "Initialization Success" << endl;

}

WiiController::~WiiController() { }

void WiiController::copyData() {
	this->moteButtonsPressed = mButtonsPressed;
	this->moteButtonsReleased = mButtonsReleased;
	this->moteButtons = mButtons;
	this->classicButtonsPressed = cButtonsPressed;
	this->classicButtonsReleased = cButtonsReleased;
	this->classicButtons = cButtons;

	/* Classic.LStick.X */
	this->lXaxis = lXa;

	/* Classic.LStick.Y */
	this->lYaxis = lYa;

	/* Classic.RStick.X */
	this->rXaxis = rXa;

	/* Classic.RStick.Y */
	this->rYaxis = rYa;

	/* Classic.LAnalog */
	this->lAnalog = lA;

	/* Classic.RAnalog */
	this->rAnalog = rA;
	toCopy = false;
}

void WiiController::handleEvent() {

	if(toCopy) {
		this->copyData();

		YClopsModel * model = (YClopsModel *) this->getModel();
		uint16_t cbuttons = this->classicButtons;
		uint16_t lax = this->lXaxis, lay = this->lYaxis;
		uint16_t rax = this->rXaxis, ray = this->rYaxis;
		uint16_t la = this->lAnalog;
		uint16_t ra = this->rAnalog;
		uint16_t mbuttons = this->moteButtons;

		//Set the speed parameters in the parent models
		this->linearSpeed = lay;
		this->angularSpeed = ray;

		if( cbuttons & CLASSIC_D_UP ) {
			LOG(INFO) << "Toggling camera data" << endl;
			model->yclops->toggleCameraDump();
		}

		if( cbuttons & CLASSIC_D_DOWN ) {
			LOG(INFO) << "Toggling lidar data" << endl;
			model->yclops->toggleLidarDump();
		}

		if( cbuttons & CLASSIC_D_LEFT ) {
			LOG(INFO) << "Toggling GPS data" << endl;
			model->yclops->toggleGpsDump();
		}

		if( cbuttons & CLASSIC_D_RIGHT ) {
			LOG(INFO) << "Toggling Compass data" << endl;
			model->yclops->toggleCompassDump();
		}

		if( cbuttons & CLASSIC_A ) {
			LOG(INFO) << "Going into Autonomous Mode" << endl;
			model->yclops->setAutonomousMode();
			model->ai->setChallenge(false);
			string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
			model->ai->setFileName(pointsFile,false);
			model->ai->setup();
		}

		if( cbuttons & CLASSIC_B ) {
			LOG(INFO) << "Going into Navigation Mode" << endl;
			model->yclops->setNavigationMode();
			model->ai->setChallenge(true);
			string pointsFile = YClopsConfiguration::instance().read_string("GLOBAL_CONFIG","POINTS_FILE","points.txt");
			model->ai->setFileName(pointsFile,false);
			model->ai->setup();
		}

		if( cbuttons & CLASSIC_X ) {
			LOG(INFO) << "Idling" << endl;
			model->yclops->useNullMotorCommand();
			model->yclops->setIdle();
			model->ai->stop();
		}

		if( cbuttons & CLASSIC_Y ) {
			LOG(INFO) << "Wii Motor Control" << endl;
			model->yclops->useWiiMotorCommand();
			model->ai->stop();
		}

		if( cbuttons & CLASSIC_L1 ) {
			LOG_WII(DEBUG4) << "Classic L1" << endl;
		}

		if( cbuttons & CLASSIC_L2 ) {
			LOG(INFO) << "Toggling Encoder data" << endl;
			model->yclops->toggleEncoderDump();
		}

		if( cbuttons & CLASSIC_R1 ) {
			LOG_WII(DEBUG4) << "Classic R1" << endl;
		}

		if( cbuttons & CLASSIC_R2 ) {
			LOG_WII(DEBUG4) << "Classic R2" << endl;
		}

		if( cbuttons & CLASSIC_SELECT ) {
			this->increaseLogLevel();
		}

		if( cbuttons & CLASSIC_HOME ) {
			LOG(DEBUG4) << "Shutdown button pressed" << endl;
			isClosing = true;
		}

		if( cbuttons & CLASSIC_SELECT ) {
			this->decreaseLogLevel();
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
			this->decreaseLogLevel();
		}

		if( mbuttons & MOTE_MINUS ) {
			LOG_WII(DEBUG4) << "Mote Minus" << endl;
			this->increaseLogLevel();
		}

		if( mbuttons & MOTE_HOME ) {
			LOG(DEBUG4) << "Shutdown button pressed" << endl;
			isClosing = true;
		}

		if( mbuttons & MOTE_A ) {
			LOG_WII(DEBUG4) << "Mote A" << endl;
		}

		if( mbuttons & MOTE_B ) {
			LOG_WII(DEBUG4) << "Mote B" << endl;
		}

		this->model->notifyObservers();
	}

}

void process_btn_mesg(struct cwiid_btn_mesg *mesg);
void process_classic_mesg(struct cwiid_classic_mesg *mesg);

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
		union cwiid_mesg mesg[], struct timespec *timestamp) {

	/*
	 * The wii controller passes multiple types of messages via bluetooth this switch
	 * statement handles them all
	 */
    for ( int i=0; i < mesg_count; i++) {
    	switch (mesg[i].type) {
    	case CWIID_MESG_BTN:
    		LOG_WII(DEBUG4) << "CWIID_MESG_BTN" << endl;

    		process_btn_mesg((struct cwiid_btn_mesg*) &mesg[i]);
    		toCopy = true;
    		break;
    	case CWIID_MESG_CLASSIC:
    		LOG_WII(DEBUG4) << "CWIID_MESG_CLASSIC" << endl;

    		process_classic_mesg((struct cwiid_classic_mesg *) &mesg[i]);
    		toCopy = true;
    		break;
    	case CWIID_MESG_ERROR:
    		LOG_WII(ERROR) << "Unknown CWIID Message" << endl;
    		break;
    	default:
    		LOG_WII(ERROR) << "Unsupported message" << endl;
    		LOG_WII(ERROR) << mesg[i].type << endl;
    		break;
    	}
    }
}

void process_btn_mesg(struct cwiid_btn_mesg *mesg) {

	/* Wiimote Button/Key Events */
	mButtonsPressed = mesg->buttons & ~mButtons;
	mButtonsReleased = ~mesg->buttons & mButtons;
	mButtons = mesg->buttons;

}

void process_classic_mesg(struct cwiid_classic_mesg *mesg) {

	/* Classic Button/Key Events */
	cButtonsPressed = mesg->buttons & ~cButtons;
	cButtonsReleased = ~mesg->buttons & cButtons;
	cButtons = mesg->buttons;

	/* Classic.LStick.X */
	lXa = mesg->l_stick[CWIID_X];

	/* Classic.LStick.Y */
	lYa = mesg->l_stick[CWIID_Y];

	/* Classic.RStick.X */
	rXa = mesg->r_stick[CWIID_X];

	/* Classic.RStick.Y */
	rYa = mesg->r_stick[CWIID_Y];

	/* Classic.LAnalog */
	lA = mesg->l;

	/* Classic.RAnalog */
	rA = mesg->r;
}
