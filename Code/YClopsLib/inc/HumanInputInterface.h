/**
 * @file HumanInputInterface.h
 * @date Mar 25, 2011
 * @author tallred3
 * @brief user input abstraction layer
 */

#ifndef HUMANINPUTINTERFACE_H_
#define HUMANINPUTINTERFACE_H_

#include <cwiid.h>

class HumanInputInterface {
public:

	/**
	 * @brief Creates a wii controller reference
	 * @return WiiController pointer
	 */
	static HumanInputInterface * createWiiController();
	static HumanInputInterface * createKeyboardController();

	/**
	 * @brief returns a pointer to the user input device
	 * @return HumanInputInterface a pointer to the human input device
	 */
	static HumanInputInterface * instance();

	/**
	 * @brief deletes the reference to the HumanInputInterface
	 */
	static void destroyInterface();

	/**
	 * @brief get the state of the mote buttons
	 * @param[out] mButtons boolean state of the mote buttons
	 */
	virtual void getMoteButtons( uint16_t & mButtons ) = 0;

	/**
	 * @brief get the state of the classic controller buttons
	 * @param[out] cButtons boolean state of the classic controller buttons
	 */
	virtual void getClassicButtons( uint16_t & cButtons ) = 0;

	/**
	 * @brief returns the state of the left analog stick on the classic controller
	 * @param[out] xAxis the value of the x axis, range of 0 to 63 with 32 at center
	 * @param[out] yAxis the value of the y axis, range of 0 to 63 with 32 at center
	 */
	virtual void getLeftStick( uint16_t & xAxis, uint16_t & yAxis ) = 0;

	/**
	 * @brief returns the state of the right analog stick on the classic controller
	 * @param[out] xAxis the value of the x axis, range 0 to 31 with 16 at center
	 * @param[out] yAxis the value of the y axis, range 0 to 31 with 16 at cetner
	 */
	virtual void getRightStick( uint16_t & xAxis, uint16_t & yAxis ) = 0;

	/**
	 * @brief returns the state of the left analog button
	 * @param[out] left the current analog value
	 */
	virtual void getLeftAnalog( uint16_t & left ) = 0;

	/**
	 * @brief returns the state of the right analog button
	 * @param[out] right the current analog value
	 */
	virtual void getRightAnalog( uint16_t & right ) = 0;

	bool getInputFlag() {return userInput;};
	void setInputFlag() {userInput = true;};
	void handleUserInput();

	static bool getClosingFlag() {return isClosing;};
	static void setClosingFlag() {isClosing = true;};

protected:
	bool userInput;
	static bool isClosing;
	static HumanInputInterface * interface;
	HumanInputInterface(): userInput(false) {};
	virtual ~HumanInputInterface() {};
private:
};

#endif /* HUMANINPUTINTERFACE_H_ */
