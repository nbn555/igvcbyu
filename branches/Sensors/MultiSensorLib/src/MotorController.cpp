/*
 * MotorController.cpp
 *
 *  Created on: Dec 11, 2010
 *      Author: tallred3
 */

#include <iostream>

#include <MotorController.h>

using namespace std;

bool MotorController::setSpeed( MotorChannel channel, int value ) {
	bool rval = false;
	switch(channel) {
	case Channel1:
		if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}

		break;
	case Channel2:
		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) ) {
			this->motor2Speed = value;
			rval = true;
		}
		break;
	case BothChannels:
		if( this->assertValidMotorRange(motor1SpeedMax, motor1SpeedMin, value) ) {
			this->motor1Speed = value;
			rval = true;
		}


		if( this->assertValidMotorRange(motor2SpeedMax, motor2SpeedMin, value) ) {
			this->motor2Speed = value;
			rval = true;
		}
		break;
	default:
		cerr << "Invalid Motor Channel" << endl;
		break;
	}

	return rval;
}

void MotorController::doProcess() {
	if( this->serialPort.isOpen() ) {
		stringstream parser;
		parser << "!M " << this->motor1Speed << " " << this->motor2Speed << "\n\r";
		this->serialPort.Write(parser.str().c_str(), parser.str().length());
	}
}

bool MotorController::assertValidMotorRange( int max, int min, int value ) const {
	return ( value > min && value < max );
}
