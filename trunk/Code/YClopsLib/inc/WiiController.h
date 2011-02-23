/**
 * @file WiiController.h
 * @date Feb 4, 2011
 * @author Thomas Eldon Allred
 */

#ifndef WIICONTROLLER_H_
#define WIICONTROLLER_H_

#include <cwiid.h>
#include <vector>
#include <iostream>

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

extern cwiid_mesg_callback_t cwiid_callback;

class WiiController {
public:
	static WiiController * create();
	static WiiController * getReference( int index = 0 );
	static void destroyReference( int index = 0 );
	static int getControllerCount();

	void getMoteButtons( uint16_t & mButtons ) { mButtons = this->moteButtons; }
	void getClassicButtons( uint16_t & cButtons ) { cButtons = this->classicButtons; }
	void getLeftStick( uint16_t & xAxis, uint16_t & yAxis ) { xAxis = this->lXaxis; yAxis = this->lYaxis; }
	void getRightStick( uint16_t & xAxis, uint16_t & yAxis ) { xAxis = this->rXaxis; yAxis = this->rYaxis; }
	void getLeftAnalog( uint16_t & left ) { left = this->lAnalog; }
	void getRightAnalog( uint16_t & right ) { right = this->rAnalog; }

private:
	virtual ~WiiController();
	WiiController();
	static WiiController* controller;

	cwiid_wiimote_t * wiiMote;

	void process_btn_mesg(struct cwiid_btn_mesg *mesg);
	void process_classic_mesg(struct cwiid_classic_mesg *mesg);
	friend cwiid_mesg_callback_t cwiid_callback;

	uint16_t motePrevButtons;
	uint16_t moteButtonsPressed;
	uint16_t moteButtonsReleased;
	uint16_t moteButtons;

	uint16_t classicButtonsPressed;
	uint16_t classicButtonsReleased;
	uint16_t classicButtons;

	uint16_t lXaxis, lYaxis;
	uint16_t rXaxis, rYaxis;
	uint16_t lAnalog, rAnalog;

};

#endif /* WIICONTROLLER_H_ */
