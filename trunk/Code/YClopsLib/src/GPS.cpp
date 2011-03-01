/*
 * GpsDevice.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: igvcbyu
 */

#include "GPS.h"
#include "logging.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;

const std::string GPS::GPSStringer::POCKETMAX = "PocketMAX";
const std::string GPS::GPSStringer::NOVATEL = "Novatel";

GPS::GPS(const int Buffer_Length):CGPSInterface(Buffer_Length), isGpggaUsed(false), isGprmcUsed(false) { //fixes potential default constructor problem
}

void GPS::loadConfiguration( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {

	std::string portName = config.read_string(sectionName,"COM_port_LIN","/dev/ttyS0");
	std::string vendor = config.read_string(sectionName, "GPS_TYPE", "Novatel" );

	if( GPSStringer::NOVATEL == vendor ) {
		this->gpsStrings = new NovatelGPSStringer( portName );
	} else if( GPSStringer::POCKETMAX == vendor ) {
		this->gpsStrings = new PocketMaxGPSStringer( portName );
	} else {
		LOG(FATAL) << "Unsupported vendor" << endl;
	}

	LOG(DEBUG3) << "Connecting to port " << portName << endl;
	this->setSerialPortName ( portName );
	this->loadConfig(config, sectionName);

	LOG(DEBUG3) << "Using " << vendor << " GPS" << endl;

	this->isGpggaUsed = config.read_bool(sectionName,"use_gga", true);

	LOG(DEBUG3) << "Using gpgga: " << (this->isGpggaUsed?"TRUE":"FALSE") << endl;

	this->isGprmcUsed = config.read_bool(sectionName,"use_rmc", false);

	LOG(DEBUG3) << "Using gprmc: " << (this->isGprmcUsed?"TRUE":"FALSE") << endl;

	this->baudRate = config.read_int(sectionName,"baudRate", 57600);

	LOG(DEBUG3) << "Using " << this->baudRate << " baud" << endl;

	this->processRate = config.read_int(sectionName,"process_rate", 1);

	LOG(DEBUG3) << "Using a process rate of " << this->processRate << endl;

	this->testLat = config.read_int(sectionName, "testLat", 0,0);

	this->testLon = config.read_int(sectionName, "testLon", 0,0);


}

void GPS::init() {
	initializeCom();

	while(!lstObs.size()) {
		doProcess();
		LOG(DEBUG3) << "Getting GPS observations" << endl;
		mrpt::system::sleep(200);
		getObservations(lstObs);
		//this->isGPS_signalAcquired()
	}

	gpsData = CObservationGPSPtr(lstObs.begin()->second);

	if(!gpsData.pointer()->has_GGA_datum)
		LOG(WARNING) << "gpsData.pointer()->has_GGA_datum fails" << endl;
	if(!gpsData.pointer()->has_RMC_datum)
		LOG(WARNING) << "gpsData.pointer()->has_RMC_datum fails" << endl;

}

void GPS::sensorProcess() {
	doProcess();
	getObservations( lstObs );

	if (lstObs.empty())
		LOG(DEBUG3) << "[Test_GPS] Waiting for data..." << endl;

	for(itObs = lstObs.begin(); itObs != lstObs.end(); itObs++){

		gpsData=CObservationGPSPtr(itObs->second);

		if(!gpsData.pointer()->has_GGA_datum)
			LOG(WARNING) << "gpsData.pointer()->has_GGA_datum fails" << endl;
		if(!gpsData.pointer()->has_RMC_datum)
			LOG(WARNING) << "gpsData.pointer()->has_RMC_datum fails" << endl;
	}
}

SensorData * GPS::getData() {
	double lat;
	double lon;
	bool valid = false;

	if(gpsData.pointer()->has_GGA_datum) {
		lat = gpsData.pointer()->GGA_datum.latitude_degrees;
		lon = gpsData.pointer()->GGA_datum.longitude_degrees;
		valid = true;
	} else if(gpsData.pointer()->has_RMC_datum) {
		lat = gpsData.pointer()->RMC_datum.latitude_degrees;
		lon = gpsData.pointer()->RMC_datum.longitude_degrees;
		valid = true;
	}

	return new GPSData( lat, lon, valid );
}

void GPS::dumpData(std::ostream & out ) const {
	out << "************" << endl;
	out << "GPS" << endl;
	out << "Connection: " << ((GPS*)(this))->isGPS_connected() << " " << "Signal: " << ((GPS*)(this))->isGPS_signalAcquired();

	static bool isValid = false;
	//static CObservationGPSPtr gpsData;
	//CGenericSensor::TListObservations				lstObs;
	//prime the pump with the first gps observation

	//this->getObservations(lstObs);
	out << " Size: " << lstObs.size() << endl;
	if(lstObs.size()) {
		isValid = true;
		//gpsData = CObservationGPSPtr(lstObs.begin()->second);
		//this->appendObservation(gpsData);//This probably wont be used
	}
	if(isValid) {
		if( gpsData->has_GGA_datum ) {
			out << "GGA: " << gpsData->GGA_datum.latitude_degrees << ", " << gpsData->GGA_datum.longitude_degrees << endl;
		}

		if( gpsData->has_RMC_datum ) {
			out << "RMC: " << gpsData->RMC_datum.latitude_degrees << ", " << gpsData->RMC_datum.longitude_degrees << endl;
		}

		cout << "Distance to Test Waypoint: " << GetDistanceToWaypoint(testLat, testLon, GetGpsLatitude(), GetGpsLongitude())
				<< " meters" << endl;

	}
	out << "************" << endl;
}

GPS::~GPS() {
	delete this->gpsStrings;
	this->gpsStrings = NULL;
}

double GPS::GetDistanceToWaypoint (double lat, double lon) {
	return AbstractNavigationInterface::haversineDistance(lat, lon, GetGpsLatitude(), GetGpsLongitude());
}

double GPS::GetDistanceToWaypoint (double lat1, double lon1, double lat2, double lon2) {
	return AbstractNavigationInterface::haversineDistance(lat1, lon1, lat2, lon2);
}

double GPS::GetGpsSpeed() {
	const double knotToMph = 0.868976242;
	if (gpsData->has_RMC_datum)
		return (gpsData->RMC_datum.speed_knots * knotToMph);
	else return 0.0;
}

double GPS::GetGpsDirection() {

	if (gpsData->has_RMC_datum)
		return gpsData->RMC_datum.direction_degrees;

	else
		LOG(DEBUG3) << "No GPS_RMC_datum to return" << endl;
	return 0.0;
}

double GPS::GetGpsLatitude() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.latitude_degrees;

	return 0.0;
}

double GPS::GetGpsLongitude() {
	if (gpsData->has_GGA_datum)
		return gpsData->GGA_datum.longitude_degrees;

	return 0.0;
}

CPoint2D GPS::GetCurrentGpsLocation() {
	CPoint2D curPos;
	curPos.m_coords[0] = GetGpsLatitude();
	curPos.m_coords[1] = GetGpsLongitude();

	return curPos;
}



//**************  PRIVATE  *****************************

void GPS::initializeCom() {
	// check if connection is already established, if not, reconnect
	//if (this->isGPS_connected()) {return; }

	LOG(DEBUG3) << "Connecting to GPS ... (./YClopsLib/GPS)" << endl;

	CSerialPort myCom(this->gpsStrings->portName);

	myCom.setConfig(this->baudRate,0,8,1,false);

	LOG(DEBUG3) << "Post open" << endl;

	stringstream inputStream;

	//clear out the commands that constantly update the gps data
	bool to = false;

	inputStream << this->gpsStrings->clearCommand << "\n\r";
	myCom.Write(inputStream.str().c_str(),inputStream.str().length());
	sleep(1);
	myCom.purgeBuffers();

	string response = myCom.ReadString( 1000, &to, "\n\r" );

	if( this->gpsStrings->clearCommandResponse != response.substr(0,this->gpsStrings->clearCommandResponse.length()) || to ) {
		LOG(FATAL) << "Misconfigured " << this->gpsStrings->vendor << " GPS" << endl;
	}

	if( this->isGpggaUsed ) {
		LOG(DEBUG3) << "Using gpgga" << endl;
		inputStream.str("");
		inputStream << this->gpsStrings->ggaCommand << this->processRate << "\n\r";
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to get a satellite reading once a second
	}

	if( this->isGprmcUsed ) {
		LOG(DEBUG3) << "Using gprmc" << endl;
		inputStream.str("");
		inputStream << this->gpsStrings->rmcCommand << this->processRate << "\n\r";
		myCom.Write(inputStream.str().c_str(),inputStream.str().length());//this will tell the gps to run the nmea rmc command once a second
	}

	myCom.close();
}
