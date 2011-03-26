/**
 * @file WiiController.h
 * @date Feb 4, 2011
 * @author Thomas Eldon Allred
 * @brief Header file for interfacing with the Wii Controller
 */

#ifndef WIICONTROLLER_H_
#define WIICONTROLLER_H_

#include <cwiid.h>
#include <vector>
#include <iostream>
#include "HumanInputInterface.h"

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

/**
 * @brief Interfaces with a wii controller
 */
class WiiController: public HumanInputInterface {
public:

	/**
	 * @brief get the state of the mote buttons
	 * @param[out] mButtons boolean state of the mote buttons
	 */
	void getMoteButtons( uint16_t & mButtons ) { mButtons = this->moteButtons; }

	/**
	 * @brief get the state of the classic controller buttons
	 * @param[out] cButtons boolean state of the classic controller buttons
	 */
	void getClassicButtons( uint16_t & cButtons ) { cButtons = this->classicButtons; }

	/**
	 * @brief returns the state of the left analog stick on the classic controller
	 * @param[out] xAxis the value of the x axis, range of 0 to 63 with 32 at center
	 * @param[out] yAxis the value of the y axis, range of 0 to 63 with 32 at center
	 */
	void getLeftStick( uint16_t & xAxis, uint16_t & yAxis ) { xAxis = this->lXaxis; yAxis = this->lYaxis; }

	/**
	 * @brief returns the state of the right analog stick on the classic controller
	 * @param[out] xAxis the value of the x axis, range 0 to 31 with 16 at center
	 * @param[out] yAxis the value of the y axis, range 0 to 31 with 16 at cetner
	 */
	void getRightStick( uint16_t & xAxis, uint16_t & yAxis ) { xAxis = this->rXaxis; yAxis = this->rYaxis; }

	/**
	 * @brief returns the state of the left analog button
	 * @param[out] left the current analog value
	 */
	void getLeftAnalog( uint16_t & left ) { left = this->lAnalog; }

	/**
	 * @brief returns the state of the right analog button
	 * @param[out] right the current analog value
	 */
	void getRightAnalog( uint16_t & right ) { right = this->rAnalog; }

private:
	virtual ~WiiController();
	WiiController();

	cwiid_wiimote_t * wiiMote;			//!<pointer to the wiiMote struct of the cwiid library

	/**
	 * @brief processes a button bluetooth message
	 * @param[in] mesg button message
	 */
	void process_btn_mesg(struct cwiid_btn_mesg *mesg);

	/**
	 * @brief processes a classic controller message
	 * @param[in] mesg a classic controller message
	 */
	void process_classic_mesg(struct cwiid_classic_mesg *mesg);

	/**
	 * @brief allows us to call process_btn_mesg and process_classic_mesg which shouldn't be public
	 */
	friend cwiid_mesg_callback_t cwiid_callback;
	friend class HumanInputInterface;

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
