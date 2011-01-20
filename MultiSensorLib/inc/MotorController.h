/*
 * MotorController.h
 *
 *  Created on: Dec 11, 2010
 *      Author: T. Eldon Allred
 *      Written for the purpose of the interfacing with the RoboteQ 2440 motor controller
 *      for the BYU 2010-2011 IGVC submission.
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <string>

#include <mrpt/hwdrivers/CSerialPort.h>

class MotorController {
public:
	static const int MOTOR_SECRET_KEY = 321654987;//Motor reset key used for sensitive operations
	enum MotorChannel{
		Channel1 = 1,
		Channel2,
		BothChannels
	};

	MotorController( std::string portName, bool enableEcho = true, int motor1Max = 1000, int motor2Max = 1000,
			int motor1Min = -1000, int motor2Min = -1000 );

	virtual ~MotorController() { };

	/**
	 * setSpeed - sets the speed for the motor control channel
	 * @param channel - the channel to change the speed { Channel1, Channel2, BothChannels }
	 * @param value - the speed at which to set the channel
	 * @return true upon success
	 */
	bool setSpeed( MotorChannel channel, int value );

	/**
	 * setEncoderCounter - sets the wheel encoder count to a specific value
	 * @param channel - the channel of the wheel encoder count to set
	 * @param value - the value to set the encoder count
	 * @return true upon success
	 */
	bool setEncoderCounter( MotorChannel channel, int value = 0 );

	/**
	 * getRelativeEncoderCount - gets the encoder count since the last time this command was used
	 * @param ch1 - the encoder count for the first channel
	 * @param ch2 - the encoder count for the second channel
	 * @return true if successful
	 */
	bool getRelativeEncoderCount( int & ch1, int & ch2 );

	/**
	 * getTemperature - gets the temperature of the heatsinks of the motor controller
	 * @param ch1 - the temperature in degrees C for the first channel
	 * @param ch2 - the temperature in degrees C for the second channel
	 * @return true if successful
	 */
	bool getTemperature( int & ch1, int & ch2 );


	/**
	 * doProcess - sends the current speed command to the motor controller
	 */
	void doProcess();

	/**
	 * emergencyStop - sends the emergency stop command to the motor controller
	 * the motor controller will not start until it is power cycled or the clearEmergencyStop
	 * function is called.
	 * @returns true upon successful transmission
	 */
	bool emergencyStop();

	/**
	 * clearEmergencyStop - clears the emergency stop state
	 * @returns true upon successful transmission
	 */
	bool clearEmergencyStop();

	/**
	 * reset - power cycles the motor controller
	 * @returns true upon successful transmission
	 */
	bool reset();

	/**
	 * setTime - sets the local time of the motor controller to the time in the parameters
	 * @param hours - the hours segment of the time
	 * @param minutes - the minutes segment of the time
	 * @param seconds - the seconds segment of the time
	 * @returns true upon successful transmission
	 */
	bool setTime( int hours, int minutes, int seconds );

	/**
	 * getMotorLimits - returns the maximum rpms for the motors
	 * @param m1Max - a reference to where to put the motor 1 max
	 * @param m2Max - a reference to where to put the motor 2 max
	 * @param m1Min - a reference to where to put the motor 1 min
	 * @param m2Min - a reference to where to put the motor 2 min
	 * @returns true
	 */
	bool getMotorLimits( int & m1Max, int & m2Max, int & m1Min, int & m2Min );

private:
	mrpt::hwdrivers::CSerialPort serialPort;//! the serial port to communicate to the motor controller
	bool echoEnabled;						//! true if the motor controller is set to echo serial commands
	int motor1Speed;						//! the current speed for the first channel motor
	int motor2Speed;						//! the current speed for the second channel motor
	const int motor1SpeedMax;				//! The hard upper limit for the first channel motor speed
	const int motor2SpeedMax;				//! The hard upper limit for the second channel motor speed
	const int motor1SpeedMin;				//! The hard lower limit for the first channel motor speed (lower limit would be the reverse speed)
	const int motor2SpeedMin;				//! The hard lower limit for the second channel motor speed

	/**
	 * assertValidMotorRange - checks the parameters if they are in a valid range
	 * @param max - the max value for the range
	 * @param min - the min value for the range
	 * @param value - the value to check
	 * @returns true if the value is in range
	 */
	bool assertValidMotorRange( int max, int min, int value ) const;

	/**
	 * assertValidVoltage - shuts off the motor controller if the voltage is out of safe operating parameters
	 */
	bool assertValidVoltage();

	/**
	 * sendCommand - low level function call to send a command to the motor controller
	 * @param command - the carriage return ended command to send to the motor controller
	 * @returns true upon successful transmission of the command
	 */
	bool sendCommand( std::string command );

	/**
	 * enableSerialEcho - turns on the echo of commands from the motor controller
	 */
	void enableSerialEcho();

	/**
	 * disableSerialEcho - turns off the echo of commands from the motor controller
	 */
	void disableSerialEcho();
};
#endif /* MOTORCONTROLLER_H_ */
