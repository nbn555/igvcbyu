/**
 * @file MotorController.h
 * @date Dec 11, 2010
 * @author T. Eldon Allred
 * @brief Written for the purpose of the interfacing with the RoboteQ 2440 motor controller
 *      for the BYU 2010-2011 IGVC submission.
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <string>

#include <mrpt/hwdrivers/CSerialPort.h>

/**
 * MotorController - interface class to the RoboteQ 2440 motor controller
 */
class MotorController {
public:
	static const int MOTOR_SECRET_KEY = 321654987;//!Motor reset key used for sensitive operations

	/**
	 * Abstract representation for a channel in the motor controller
	 */
	enum MotorChannel{
		Channel1 = 1,
		Channel2 = 2,
		BothChannels = 3
	};

	/**
	 * Set of possible status states in the motor controller
	 */
	enum MotorControllerStatus{
		overheat,
		overvoltage,
		undervoltage,
		short_circuit,
		emergency_stop,
		sepex_excitation_fault,
		EEPROM_fault,
		configuration_fault
	};

	/**
	 * MotorController constructor
	 * @param portName - the serial port over which the commands will be sent
	 * @param enableEcho - set to true to check error for errors from the motor controller
	 * @param motor1Max - the maximum power for channel1 defined on the interval [1000,-1000] where -1000 is full reverse
	 * @param motor2Max - the maximum power for channel2 defined on the interval [1000, -1000] where -1000 is full reverse
	 * @param motor1Min - the minimum power for channel1
	 * @param motor2Min - the minimum power for channel2
	 */
	MotorController( std::string portName, bool enableEcho = true, int motor1Max = 1000, int motor2Max = 1000,
			int motor1Min = -1000, int motor2Min = -1000 );

	/**
	 * Class destructor
	 */
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
	 * getEncoderSpeed - returns the speed of the encoder for the given channel
	 * @param channel - the motor channel to query
	 * @param speed - the speed of the encoder
	 * @returns - true upon success
	 */
	bool getEncoderSpeed( MotorChannel channel, int & speed );

	/**
	 * setEncoderPPR - sets the pulses per revolution for the encoder on the motor controller
	 * @param ppr - the number of pulses per revolution on the wheel encoder
	 * @returns true upon success
	 */
	bool setEncoderPPR( int ppr );

	/**
	 * setSerialWatchDogTimer - will set the serial watchdog timer to be equal
	 * to the parameter where it is measured in ms
	 * @param time - the length of the watchdog timer as measured in ms
	 * @return true upon success
	 */
	bool setSerialWatchDogTimer( int time = 1000 );

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
	 * getVoltages - gets the voltages from the battery, internal motor driver
	 * and the 5 volt usb out.
	 * @param driverVolt - the motor driver output voltages as measured in decivolts
	 * @param batteryVolt - the battery voltage as measured in decivolts
	 * @param v5out - the voltage of the usb as measured in millivolts
	 * @return true upon success.
	 */
	bool getVoltages( int & driverVolt, int & batteryVolt, int & v5out )
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
	 * failSafeReset - power cycles the motor controller and resets
	 * values to thier default factory settings
	 */
	bool failSafeReset();

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
	int faultFlag							//! The fault flag status for the motor controller
	MotorControllerStatus status;			//! The status of the motor controller

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

	/**
	 * getFaultFlags returns the status of any faults that may occour in the motor controller
	 */
	void getFaultFlags();
};
#endif /* MOTORCONTROLLER_H_ */
