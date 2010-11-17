/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include <iostream>

#include "GPS.h"

using namespace std;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

GPS::GPS(const int Buffer_Length):CGPSInterface(Buffer_Length), isGpggaUsed(false), isGprmcUsed(false) { //fixes potential default constructor problem
}

GPS::~GPS() {

}

//This function ought to be called after loadConfig but before initialize
void GPS::initConfig( mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);
	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);
	this->baudRate = config.read_int(sectionName,"baudRate", 9600);
	this->portName = config.read_string(sectionName,"COM_port_LIN","/dev/ttyS0");

}

void GPS::initialize() {
	string writeValue;

	cout << "here" << endl;
	CSerialPort myCom(this->portName);
	myCom.setConfig(this->baudRate,0,8,1,false);
	cout << "Post open" << endl;

	/*
	//This code segment demonstrates how to write a command and read data from the GPS
	bool tout = false;
	writeValue = "$jshow\n\r";//Commands must be terminated with \n\r
	myCom.WriteBuffer(writeValue.c_str(), writeValue.length());

	while(!tout) {
		string readString = myCom.ReadString(100,&tout,"\n\r");
		cout << readString << endl;
	}
	*/

	if( this->isGpggaUsed ) {
		writeValue = "$jasc,gpgga,1\n\r";
		myCom.Write(writeValue.c_str(),writeValue.length());//this will tell the old gps to get a satellite reading once a second
	}

	if( this->isGprmcUsed ) {
		writeValue = "$jasc,gprmc,1\n\r";
		myCom.Write(writeValue.c_str(),writeValue.length());//this will tell the old gps to run the nmea rmc command once a second
	}

	myCom.close();

}
