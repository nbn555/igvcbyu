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
	static std::vector<WiiController*> controllers;

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
