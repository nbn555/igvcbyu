/**
 * @file WiiController.cpp
 * @date Feb 4, 2011
 * @author Thomas Eldon Allred
 */

#include "WiiController.h"
#include "logging.h"

#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <unistd.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

using namespace std;

WiiController * WiiController::controller = NULL;
cwiid_mesg_callback_t cwiid_callback;

WiiController * WiiController::create() {

	if( NULL == WiiController::controller )
		WiiController::controller = new WiiController();
	return WiiController::controller;
}

WiiController * WiiController::getReference( int index ) {
		return WiiController::create();
}

void WiiController::destroyReference( int index ) {
	delete WiiController::controller;
}

int WiiController::getControllerCount() {
	return 1;
}

WiiController::WiiController() {

	bdaddr_t bdaddr = *BDADDR_ANY;

	this->wiiMote = NULL;
	static int sval = 0;
	LOG(DEBUG2) << "In constructor " << ++sval << endl;

	int dev_id = -1;
	bool isBluetoothFound = false;
	while(!isBluetoothFound) {
		LOG(INFO) << "Searching for Bluetooth interface" << endl;
		if((dev_id = hci_get_route(NULL)) == -1) {
			LOG(WARNING) << "Couldn't find Bluetooth interface" << endl;
		} else isBluetoothFound = true;
	}

	LOG(INFO) << "Bluetooth found" << endl;

	cout << "Press buttons 1 + 2 to put WiiMote in Discoverable mode" << endl;
	while(-1 ==  cwiid_find_wiimote(&bdaddr, 5)) {
		//cout << "Waiting to "
	}

	while(!this->wiiMote) {
		this->wiiMote = cwiid_open(&(bdaddr),CWIID_FLAG_MESG_IFC);
		if(!this->wiiMote)
			LOG(WARNING) << "Connect Fail Retrying" << endl;
	}

	cout << "Connected WiiMote" << endl;

	cwiid_set_mesg_callback(this->wiiMote, &cwiid_callback);
	cwiid_command(this->wiiMote, CWIID_CMD_RPT_MODE, CWIID_RPT_BTN | CWIID_RPT_CLASSIC );

	LOG(DEBUG2) << "Initialization success" << endl;
}

WiiController::~WiiController() {
	cwiid_close(this->wiiMote);
}

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
		union cwiid_mesg mesg[], struct timespec *timestamp) {

	int index = 0;
//	const int controllerCnt = WiiController::getControllerCount();
//	for( int i = 0; i < controllerCnt; i++ ) {
//		WiiController ref = WiiController::getReference(i);
//		if(cwiid_get_id(wiimote) == cwiid_get_id(ref.wiiMote)) {
//			index = i;
//			break;
//		}
//	}

	WiiController * control = WiiController::getReference(index);
    for ( int i=0; i < mesg_count; i++) {
    	switch (mesg[i].type) {
    	case CWIID_MESG_BTN:
    		LOG(DEBUG4) << "CWIID_MESG_BTN" << endl;

    		control->process_btn_mesg((struct cwiid_btn_mesg*) &mesg[i]);
     		raise(SIGUSR1);
    		break;
    	case CWIID_MESG_CLASSIC:
    		LOG(DEBUG4) << "CWIID_MESG_CLASSIC" << endl;
    		LOG(DEBUG4) << control << endl;

    		control->process_classic_mesg((struct cwiid_classic_mesg *) &mesg[i]);
    		raise(SIGUSR1);
    		break;
    	case CWIID_MESG_ERROR:
    		LOG(ERROR) << "Unknown CWIID Message" << endl;
    		break;
    	default:
    		LOG(ERROR) << "Unsupported message" << endl;
    		LOG(ERROR) << mesg[i].type << endl;
    		break;
    	}
    }
}

void WiiController::process_btn_mesg(struct cwiid_btn_mesg *mesg) {

	/* Wiimote Button/Key Events */
	this->moteButtonsPressed = mesg->buttons & ~this->moteButtons;
	this->moteButtonsReleased = ~mesg->buttons & this->moteButtons;
	this->moteButtons = mesg->buttons;

}

void WiiController::process_classic_mesg(struct cwiid_classic_mesg *mesg) {

	/* Classic Button/Key Events */
	this->classicButtonsPressed = mesg->buttons & ~this->classicButtons;
	this->classicButtonsReleased = ~mesg->buttons & this->classicButtons;
	this->classicButtons = mesg->buttons;

	/* Classic.LStick.X */
	this->lXaxis = mesg->l_stick[CWIID_X];

	/* Classic.LStick.Y */
	this->lYaxis = mesg->l_stick[CWIID_Y];

	/* Classic.RStick.X */
	this->rXaxis = mesg->r_stick[CWIID_X];

	/* Classic.RStick.Y */
	this->rYaxis = mesg->r_stick[CWIID_Y];

	/* Classic.LAnalog */
	this->lAnalog = mesg->l;

	/* Classic.RAnalog */
	this->rAnalog = mesg->r;

}
