/*
 * MotorController.h
 *
 *  Created on: Dec 11, 2010
 *      Author: tallred3
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <mrpt/hwdrivers/CSerialPort.h>

class MotorController {
public:
	enum MotorChannel{
		Channel1 = 1,
		Channel2,
		BothChannels
	};

	MotorController( int motor1Max = 1000, int motor2Max = 1000,
			int motor1Min = -1000, int motor2Min = -1000 ):
		motor1Speed(0), motor2Speed(0),
		motor1SpeedMax(motor1Max), motor2SpeedMax(motor2Max),
		motor1SpeedMin(motor1Min), motor2SpeedMin(motor2Min) { };

	virtual ~MotorController() { };

	bool setSpeed( MotorChannel channel, int value );
	void doProcess();
private:
	mrpt::hwdrivers::CSerialPort serialPort;
	int motor1Speed;
	int motor2Speed;
	int motor1SpeedMax;
	int motor2SpeedMax;
	int motor1SpeedMin;
	int motor2SpeedMin;

	bool assertValidMotorRange( int max, int min, int value ) const;
};
#endif /* MOTORCONTROLLER_H_ */
