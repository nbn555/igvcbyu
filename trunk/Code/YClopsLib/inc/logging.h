/*
 * @file logging.h
 * @date Feb 17, 2011
 * @author Thomas Eldon Allred
 * @brief Logging Header
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#include <sstream>
#include <iostream>

typedef enum LOG_LEVEL{
	DISABLE = 8,//!<Turn off logging completely
	FATAL 	= 7,//!<Fatal or irrecoverable errors
	ERROR 	= 6,//!<Error Conditions
	WARNING	= 5,//!<Potential problems
	INFO	= 4,//!<Standard output
	DEBUG	= 3,//!<The general idea is the more function calls to get there the higher the debug level
	DEBUG2	= 2,
	DEBUG3	= 1,
	DEBUG4	= 0
} LOG_LEVEL;

#define COMPASS_LOG	(0x00000001)
#define CAMERA_LOG	(0x00000002)
#define GPS_LOG		(0x00000004)
#define LIDAR_LOG	(0x00000008)
#define MOTOR_LOG	(0x00000010)
#define POSE_LOG	(0x00000020)
#define WII_LOG		(0x00000040)
#define ENCODER_LOG	(0x00000080)
#define AI_LOG		(0x00000100)
#define GENERIC_LOG	(0x80000000)
#define ALL_LOG		(0xFFFFFFFF)

/**
 * @brief macro allows us to not have to dump data into a stream if we aren't the log level is to low
 * @warning Don't put non trivial function calls in the output stream or they wont be called when the logging isn't output
 * @param[in] level the level at which to print the output
 * @param[in] stream the input steam
 */
#define LOG_STREAM(level,stream) \
((level < Log::ReportingLevel()) || (stream & Log::GetReportStreamBits())) ? : Log().Get(level)

#define LOG_CAMERA(level)	LOG_STREAM(level,CAMERA_LOG)

#define LOG_COMPASS(level)	LOG_STREAM(level,COMPASS_LOG)

#define LOG_GPS(level)		LOG_STREAM(level,GPS_LOG)

#define LOG_LIDAR(level)	LOG_STREAM(level,LIDAR_LOG)

#define LOG_MOTOR(level)	LOG_STREAM(level,MOTOR_LOG)

#define LOG_POSE(level)		LOG_STREAM(level,POSE_LOG)

#define LOG_WII(level)		LOG_STREAM(level,WII_LOG)

#define LOG_ENCODER(level)	LOG_STREAM(level,ENCODER_LOG)

#define LOG_AI(level)		LOG_STREAM(level,AI_LOG)

#define LOG(level)			LOG_STREAM(level,GENERIC_LOG)

/**
 * @brief Implementes logging functionality
 */
class Log {
public:
	Log() {};
	virtual ~Log();
	std::stringstream & Get(LOG_LEVEL level = INFO);
public:

	/**
	 * @brief returns a reference to the current reporting level
	 * @return LOG_LEVEL& a reference of the current logging level
	 */
	static LOG_LEVEL & ReportingLevel();

	/**
	 * @brief returns the current of the logging output streams
	 * @return unsigned int which is a bit string of the logging streams I don't remember the logic levels of the bits
	 */
	static unsigned int GetReportStreamBits();

	/**
	 * @brief sets which logging streams are actually used.  Eg if the corresponding bit value is high the log is used (I think).
	 * @param s the string of bits to represent which logs are used
	 */
	static void SetReportStreamBits( unsigned int s );

	/**
	 * @brief toggles if the time stamp for a log is displayed or not
	 * @param ts the boolean value if the time stamp is displayed or not
	 */
	static void SetTimeStampDisplay( bool ts );

	/**
	 * @brief sets which reporting level is displayed
	 */
	static void SetReportLevel( LOG_LEVEL level = INFO );

	/**
	 * @brief sets the output of the logging stream
	 * @param[in] out the output stream pointer use &cout for console
	 */
	static void SetLogFile( std::ostream * out );

	/**
	 * @brief returns a pointer to the current output stream
	 * @return std::ostream* a pointer to the current output stream
	 */
	static std::ostream * GetOStream() { return outputStream; };

	/**
	 * @brief converst the log level to a string
	 * @param[in] level the logging level to convert to a string
	 * @return a string representation of the level
	 */
	static std::string ToString( LOG_LEVEL level );
protected:
	std::stringstream os; //!< The output stream where to write the log data
protected:
	static std::string GetTime();		//!<returns a string formatted with of the time stamp
private:
	Log(const Log&);
	Log & operator = (const Log&);
private:
	LOG_LEVEL messageLevel;				//!<The log level of the current message
private:
	static LOG_LEVEL logLevel;			//!<The current logging threshold
	static unsigned int streamBits;		//!<A bit string for which logs we are using
	static bool timestamp;				//!<if we are using time stamps or not
	static std::ostream * outputStream;	//!< a pointer to the output stream we are using

};

#endif /* LOGGING_H_ */
