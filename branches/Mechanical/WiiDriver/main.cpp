/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *      Author: igvcbyu
 */

#include <iostream>
#include <stdlib.h>
#include <signal.h>

#include <mrpt/utils/CConfigFile.h>

#include "MotorController.h"
#include "MotorCommand.h"
#include "Compass.h"
#include "WiiController.h"

#include <mrpt/hwdrivers/CSerialPort.h>

using namespace mrpt::utils;
using namespace std;

void signal_handler( int signum );

int main( int argc, char** argv ) {

	//create the config file
	CConfigFile configFile;

	//test if the config file is passed in
	if(argc < 2) {
		//if not print exit message and die
		cout << "Usage: wiiDriver file.ini" << endl;
		exit(EXIT_FAILURE);
	}else {
		//if so load the config file
		configFile.setFileName(string(argv[1]));
	}

	//Set up the SIGUSR1 so we know when a button is pressed
	signal(SIGUSR1, signal_handler);

	//Initialize the WiiController
	WiiController::create();

	//Set the motor controller to connect to the port name in the config file
	MotorController::setPortName( configFile.read_string("MOTOR", "COM_port_LIN", "/dev/ttyS1" ) );

	MotorCommandInterface * mci = new DualMotorCommand();

	while(1) {

		mci->doProcess();
		sleep(1);

	}

	return 0;
}

void signal_handler( int signum ) {

	if( SIGUSR1 == signum ) {
		uint16_t cbuttons;
		WiiController * controller = WiiController::getReference();

		controller->getClassicButtons(cbuttons);

		cout << "classic buttons ";
		for( uint16_t bitMask = (1 << (sizeof(bitMask)*8-1)); bitMask; bitMask >>= 1) {
			if( bitMask & cbuttons ) {
				cout << '1';
			} else {
				cout << '0';
			}
		}
		cout << endl;

		cout << "Classic l analog stick ";
		uint16_t lax, lay;

		controller->getLeftStick(lax,lay);
		cout << lax << "," << lay << endl;

		cout << "Classic r analog stick ";
		uint16_t rax, ray;

		controller->getRightStick(rax,ray);
		cout << rax << "," << ray << endl;

		cout << "Classic analog l ";
		uint16_t la;
		controller->getLeftAnalog(la);
		cout << la << endl;

		cout << "Classic analog r ";
		uint16_t ra;
		controller->getRightAnalog(ra);
		cout << ra << endl;

		uint16_t mbuttons;
		controller->getMoteButtons(mbuttons);

		cout << "Mote buttons ";
		for( uint16_t bitMask = (1<<(sizeof(bitMask)*8-1));bitMask;bitMask>>=1){
			if(bitMask & mbuttons) {
				cout << '1';
			} else {
				cout << '0';
			}
		}
		cout << endl;
	}
}
