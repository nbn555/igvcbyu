/**
 * @file WiiController.h
 * @date Mar 29, 2011
 * @author tallred3
 * @brief 
 */

#ifndef WIICONTROLLER_H_
#define WIICONTROLLER_H_

#include <cwiid.h>

#include "YClopsController.h"

#define CLASSIC_D_UP	(0x0001)
#define CLASSIC_D_LEFT	(0x0002)
#define CLASSIC_R2		(0x0004)
#define CLASSIC_X		(0x0008)
#define CLASSIC_A		(0x0010)
#define CLASSIC_Y		(0x0020)
#define CLASSIC_B		(0x0040)
#define CLASSIC_L2		(0x0080)
#define CLASSIC_R1		(0x0200)
#define CLASSIC_START	(0x0400)
#define CLASSIC_HOME	(0x0800)
#define CLASSIC_SELECT	(0x1000)
#define CLASSIC_L1		(0x2000)
#define CLASSIC_D_DOWN	(0x4000)
#define CLASSIC_D_RIGHT	(0x8000)

#define MOTE_2			(0x0001)
#define MOTE_1			(0x0002)
#define MOTE_B			(0x0004)
#define MOTE_A			(0x0008)
#define MOTE_MINUS		(0x0010)
#define MOTE_HOME		(0x0080)
#define MOTE_D_LEFT		(0x0100)
#define MOTE_D_RIGHT	(0x0200)
#define MOTE_D_DOWN		(0x0400)
#define MOTE_D_UP		(0x0800)
#define MOTE_PLUS		(0x1000)

extern cwiid_mesg_callback_t cwiid_callback; //!<call back handler for interfacing messages from the wii controller

class WiiController: public YClopsController {
public:
	WiiController(util::Observable * m, MVC::AbstractView * v);
	virtual ~WiiController();
	void handleEvent();

private:
	void copyData();
	cwiid_wiimote_t * wiiMote;			//!<pointer to the wiiMote struct of the cwiid library
	friend cwiid_mesg_callback_t cwiid_callback;

	uint16_t motePrevButtons;		//!<The previous state of the mote buttons
	uint16_t moteButtonsPressed;	//!<The true for which buttons were just pressed
	uint16_t moteButtonsReleased;	//!<True for which buttons were just released
	uint16_t moteButtons;			//!<true for which buttons are pressed

	uint16_t classicButtonsPressed;	//!<True for which buttons were just pressed
	uint16_t classicButtonsReleased;//!<True for which buttons were just released
	uint16_t classicButtons;		//!<True for the state of the pressed buttons

	uint16_t lXaxis, lYaxis;		//!<classic controller left stick state
	uint16_t rXaxis, rYaxis;		//!<classic controller right stick state
	uint16_t lAnalog, rAnalog;		//!<classic controller analog state

};

#endif /* WIICONTROLLER_H_ */
