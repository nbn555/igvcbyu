/*
 * SensorData.h
 *
 *  Created on: Feb 24, 2011
 *      Author: tallred3
 */

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include <ctime>
#include <mrpt/slam.h>

class SensorData {
public:
	SensorData(): t(time(NULL)) { };
	virtual ~SensorData() {};
	time_t t;
};

class GPSData: public SensorData {
public:
	GPSData( double latitude, double longitude, bool valid ): latitude(latitude), longitude(longitude), valid(valid) {};
	virtual ~GPSData() {};

	double latitude;
	double longitude;
	bool valid;

};

class CompassData: public SensorData {
public:
	CompassData( double yaw, double pitch, double roll, bool yawValid, bool pitchValid, bool rollValid, bool degrees ):
		yaw(yaw), pitch(pitch), roll(roll), yawValid(yawValid), pitchValid(pitchValid), rollValid(rollValid), degrees(degrees) {}
	virtual ~CompassData() {};

	double yaw;
	double pitch;
	double roll;

	bool yawValid;
	bool pitchValid;
	bool rollValid;

	bool degrees;

};

class EncoderData: public SensorData {
public:
	EncoderData( int leftCount, int rightCount, int leftCountAbsolute, int rightCountAbsolute, int leftSpeed, int rightSpeed ): leftCount(leftCount), rightCount(rightCount), leftCountAbsolute(leftCountAbsolute), rightCountAbsolute(rightCountAbsolute), leftSpeed(leftSpeed), rightSpeed(rightSpeed) {};
	virtual ~EncoderData() {};
	int leftCount;
	int rightCount;
	int leftCountAbsolute;
	int rightCountAbsolute;
	int leftSpeed;
	int rightSpeed;
};

class CameraData: public SensorData {
public:
	CameraData(mrpt::slam::CSimplePointsMap map) : map(map) {};
	virtual ~CameraData() {};
	mrpt::slam::CSimplePointsMap map;
};

class LidarData: public SensorData {
public:
	LidarData() {};
	virtual ~LidarData() {};
};

#endif /* SENSORDATA_H_ */
