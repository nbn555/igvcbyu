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
#include <stdarg.h>
#include <cstring>

#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CConfigFile.h>
#include <iostream>
#include "logging.h"
#include <semaphore.h>



/**
 * @brief Interface class to the RoboteQ 2440 motor controller
 * @TODO implement PID controller constants commands in initialization
 */
class MotorController {
public:
	static const int MOTOR_SECRET_KEY = 321654987;	//!<Motor reset key used for sensitive operations
	static const std::string MOTOR_BAD_COMMAND;		//!<used in error checking currently should be '-'

	/**
	 * @brief returns a reference to the motor controller
	 * @return a pointer to the motor controller
	 */
	static MotorController * instance();

	/**
	 * @brief Abstract representation for a channel in the motor controller
	 */
	enum MotorChannel{
		Channel1 = 1,
		Channel2 = 2,
		BothChannels = 3
	};

	/**
	 * @brief Class destructor
	 */
	virtual ~MotorController() {
		sem_destroy(&(this->serialPortSem)); //!<release the the pthread semaphore
	};

	/**
	 * @brief sends the current speed command to the motor controller
	 */
	void doProcess();
	void saveConfig();

	//////////////////////////////////////////////////////////////////
	//Motor Controller Runtime commands
	//////////////////////////////////////////////////////////////////

	/**
	 * @brief sets a limit to the acceleration for the motor controller
	 * @param[in] channel the motor channel for which to set the acceleration
	 * @param[in] acceleration the number of rpms per second of acceleration precise
	 * enough to one tenth of an rpm
	 * @return bool true upon success
	 * @todo implement setAcceleration
	 */
	bool setAcceleration(MotorChannel channel, double acceleration);

	/**
	 * @brief sets a limit for how fast the motor controller will slow down
	 * a channel
	 * @param[in] channel the motor channel for which to set the acceleration
	 * @param[in] deceleration the number of rpms per second of deceleration precise to one
	 * tenth of an rpm
	 * @return bool true upon success
	 * @todo implement setDeceleration
	 */
	bool setDeceleration(MotorChannel channel, double deceration);

	/**
	 * @brief sends the emergency stop command to the motor controller
	 * the motor controller will not start until it is power cycled or the clearEmergencyStop
	 * function is called.
	 * @return bool true upon successful transmission
	 */
	bool emergencyStop();

	/**
	 * @brief clears the emergency stop state
	 * @return true upon successful transmission
	 */
	bool clearEmergencyStop();

	/**
	 * @brief sets the speed for the motor control channel
	 * @param[in] channel the channel to change the speed { Channel1, Channel2, BothChannels }
	 * @param[in] value the speed at which to set the channel
	 * @return bool true upon success
	 */
	bool setSpeed( MotorChannel channel, int value );

	/**
	 * @brief sets the wheel encoder count to a specific value
	 * @param[in] channel the channel of the wheel encoder count to set
	 * @param[in] value the value to set the encoder count
	 * @return true upon success
	 */
	bool setEncoderCounter( MotorChannel channel, int value = 0 );


	bool setEncoderUsage( int value = 1 ); //0 don't use the encoder, 1 use for feedback, 2 use for command, don't know what command does
	bool setMixingMode( int value );
	bool restoreMixingMode();
	bool setOperatingMode( int value );
	bool restoreOperatingMode();

	//////////////////////////////////////////////////////////////////
	//Motor Controller Runtime queries
	//////////////////////////////////////////////////////////////////

	/**
	 * getMotorAmps - gets the amps flowing through the motors
	 * @param motor1Amps - the number of amps flowing through the first channel motor
	 * @param motor2Amps - the number of amps flowing through the second channel motor
	 * @return true upon success
	 */
	bool getMotorAmps( double & motor1Amps, double & motor2Amps );

	/**
	 * getAnalogInputs - gets the number of millivolts on the given analog input pin
	 * @param input - the input pin to query
	 * @param value - reference to the number of millivolts on the analog input pin
	 * @return true upon success
	 */
	bool getAnalogInputs( int input, int & value );

	/**
	 * getBatteryAmps - gets the number of deciamps flowing from the batteries
	 * @param motor1Amps - the number of deciamps flowing into the first motor channel
	 * @param motor2Amps - the number of deciamps flowing into the second motor channel
	 * @return true upon success
	 */
	bool getBatteryAmps( double & motor1Amps, double & motor2Amps );

	/**
	 * getAbsoluteEncoderCount - gets the absolute number of encoder counts as a 32bit signed number
	 * @param channel1Count - reference to the encoder count for channel 1
	 * @param channel2Count - reference to the encoder count for channel 2
	 * @return true upon success
	 */
	bool getAbsoluteEncoderCount( int & ch1, int & ch2 );

	/**
	 * getRelativeEncoderCount - gets the encoder count since the last time this command was used
	 * @param ch1 - the encoder count for the first channel
	 * @param ch2 - the encoder count for the second channel
	 * @return true if successful
	 */
	bool getRelativeEncoderCount( int & ch1, int & ch2 );
	/**
	 * getEncoderSpeed - returns the speed of the encoder for the given channel
	 * @param channel - the motor channel to query
	 * @param speed - the speed of the encoder in RPMs
	 * @returns - true upon success
	 *
	 */
	bool getEncoderSpeeds( int & speed1, int & speed2 );

	/**
	 * getTemperature - gets the temperature of the heatsinks of the motor controller
	 * @param ch1 - the temperature in degrees C for the first channel
	 * @param ch2 - the temperature in degrees C for the second channel
	 * @param ic  - the temperature in degrees C of the internal IC
	 * @return true if successful
	 */
	bool getTemperature( int & ch1, int & ch2, int & ic );

	/**
	 * getTime - returns the time of the motor controller
	 * @param hours - reference to the number of hours
	 * @param minutes - reference to the number of minutes
	 * @param seconds - reference to the number of seconds
	 * @return true upon success
	 */
	bool getTime( int & hours, int & minutes, int & seconds );

	/**
	 * getVoltages - gets the voltages from the battery, internal motor driver
	 * and the 5 volt usb out.
	 * @param driverVolt - the motor driver output voltages as measured in volts
	 * @param batteryVolt - the battery voltage as measured in volts
	 * @param v5out - the voltage of the usb as measured in volts
	 * @return true upon success.
	 */
	bool getVoltages( double & driverVolt, double & batteryVolt, double & v5out );

	//////////////////////////////////////////////////////////////////
	//Motor Controller Maintenance commands
	//////////////////////////////////////////////////////////////////

	/**
	 * loadEEPROMSettings - loads setting set in the eeprom back into ram and activates the settings
	 * @return true upon success
	 */
	bool loadEEPROMSettings();

	/**
	 * failSafeReset - power cycles the motor controller and resets
	 * values to their default factory settings
	 */
	bool failSafeReset();

	/**
	 * saveConfigSettings - saves the current settings in ram to the EEPROM
	 * DON'T Save while motors are running
	 * @return true upon success
	 */
	bool saveConfigSettings();

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


	//////////////////////////////////////////////////////////////////
	//Motor Controller Read/Set configuration commands
	//////////////////////////////////////////////////////////////////

	/**
	 * SetCommandLinearity - This parameter is used for applying an exponential or a
	 * logarithmic transformation on the command input, regardless of its source
	 * (serial, pulse or analog). There are 3 exponential and 3 logarithmic choices.
	 * Exponential correction make the commands change less at the beginning and
	 * become stronger at the end of the command input range. The logarithmic
	 * correction will have a stronger effect near the start and lesser effect near
	 * the end. The linear selection causes no change to the input. A linearity
	 * transform is also available for all analog and pulse inputs. Both can be
	 * enabled although in most cases, it is best to use the Command Linearity
	 * parameter for modifying command profiles.
	 * 0 : linear (no change)
	 * 1 : exp weak
	 * 2 : exp medium
	 * 3 : exp strong
	 * 4 : log weak
	 * 5 : log medium
	 * 6 : log strong
	 */
	bool setCommandLinearity(int linearity );

	/**
	 * setCommandPriority - This parameter contains up to 3 variables and is used to set
	 * which type of command in priority the controller will respond to and in which order.
	 * The first item is the first priority, second – second priority, third – third
	 * priority. Each priority item is then one of the three command modes: Serial,
	 * Analog or RC Pulse.
	 * 1 : Serial
	 * 2 : RC
	 * 3 : Analog
	 * @param signal - the singal to set to the priority
	 * @param priority - the priority level to be set for the signal
	 * @return true upon success
	 */
	bool setCommandPriority( int signal, int priority );

	/**
	 * setSerialWatchDogTimer - will set the serial watchdog timer to be equal
	 * to the parameter where it is measured in ms
	 * @param time - the length of the watchdog timer as measured in ms
	 * @return true upon success
	 */
	bool setSerialWatchDogTimer( int time );

	/**
	 * setEncoderPPR - sets the pulses per revolution for the encoder on the motor controller
	 * @param ppr - the number of pulses per revolution on the wheel encoder
	 * @returns true upon success
	 */
	bool setEncoderPPR( int ppr );

	/**
	 * setOverVoltageLimit - sets the maximum voltage value for the power stage.  Higher voltages
	 * will cause the motor controller to turn off and signal an overvoltage fault
	 * @param limit - the upper limit of the power stage voltage measured in volts
	 * @return true upon success
	 */
	bool setOverVoltageLimit( double limit );

	/**
	 * setUnderVoltageLimit - sets the minimum voltage value for the power stage.
	 * Lower voltages will cause the motor controller to turn off and signal an undervoltage
	 * fault.
	 * @param limit - the lower limit of the power stage voltage measured in volts
	 * @return true upon success
	 */
	bool setUnderVoltageLimit( double limit );

	/**
	 * setShortCircuitDetectionThreshold - Short Circuit Detection Threshold
	 * This configuration parameter sets the threshold level for the short
	 * circuit detection. There are 4 sensitivity levels from 0 to 3.
	 * 0 : Very high sensitivity
	 * 1 : Medium sensitivity
	 * 2 : Low sensitivity
	 * 3 : Short circuit protection disabled
	 * Using a value of 3 will cause an error
	 * @param value - the sensitivity of the short circuit detection threshold to be used
	 * @return true upon success
	 */
	bool setShortCircuitDetectionThreshold( int value );

	/**
	 * getMotorLimits - returns the maximum rpms for the motors
	 * @param m1Max - a reference to where to put the motor 1 max
	 * @param m2Max - a reference to where to put the motor 2 max
	 * @param m1Min - a reference to where to put the motor 1 min
	 * @param m2Min - a reference to where to put the motor 2 min
	 * @returns true
	 */
	bool getMotorLimits( int & m1Max, int & m2Max, int & m1Min, int & m2Min );

	bool setLoopMode( MotorChannel channel, std::string & loopMode );


private:
	/**
	 * MotorController constructor
	 * @param portName - the serial port over which the commands will be sent
	 * @param enableEcho - set to true to check error for errors from the motor controller
	 * @param motor1Max - the maximum power for channel1 defined on the interval [1000,-1000] where -1000 is full reverse
	 * @param motor2Max - the maximum power for channel2 defined on the interval [1000, -1000] where -1000 is full reverse
	 * @param motor1Min - the minimum power for channel1
	 * @param motor2Min - the minimum power for channel2
	 */
	MotorController();

	mrpt::hwdrivers::CSerialPort serialPort;//!< the serial port to communicate to the motor controller
	bool echoEnabled;						//!< true if the motor controller is set to echo serial commands
	int motor1Speed;						//!< the current speed for the first channel motor
	int motor2Speed;						//!< the current speed for the second channel motor
	const int motor1SpeedMax;				//!< The hard upper limit for the first channel motor speed
	const int motor2SpeedMax;				//!< The hard upper limit for the second channel motor speed
	const int motor1SpeedMin;				//!< The hard lower limit for the first channel motor speed (lower limit would be the reverse speed)
	const int motor2SpeedMin;				//!< The hard lower limit for the second channel motor speed
	int faultFlagVector;					//!< The fault flag status for the motor controller
	int mixingMode;							//!< The how a !M command is interpreted.
	int currentMixingMode;					//!< How the !M command is currently interpreted
	int operatingMode;						//!< The configuration for the operating mode (open, closed, position)
	int currentOperatingMode;				//!< The current mode the controller is set to
	sem_t serialPortSem;					//!< A semaphore to protect the serialPort
	bool permissiveMode;					//!< True if no error checking is desired
protected:
	static MotorController * mc;			//!< The pointer to the instance of the motor controller

private:
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
	bool sendCommand( const std::string command, const std::string expectedResponse, std::string * response, bool permissive );

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
	bool getFaultFlags();

	/**
	 * getStatusFlags returns the status of the motor controller as a bit vector
	 */
	bool getStatusFlags();

	/**
	 * getControlUnitType - checks to see if the attached roboteq control unit is supported
	 * @return true upon supported control unit
	 */
	bool getControlUnitType();

	/**
	 * clearBufferHistory - clears the command history buffer
	 * @return true upon success
	 */
	bool clearBufferHistory();

	bool responseParser( std::string response, int num, ... ) {
		va_list args;
		va_start( args, num );
		std::stringstream responseParser( response );

		int* tmp = va_arg( args, int*);
		responseParser >> *tmp;
		for( int i = 1; i < num; i++) {
			responseParser.get(); //Parse off the :
			tmp = va_arg( args, int*);
			responseParser >> *tmp;
		}

		va_end(args);
		return true;
	}
};

#endif /* MOTORCONTROLLER_H_ */
