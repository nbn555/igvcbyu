/*
 * @file logging.h
 * @date Feb 17, 2011
 * @author Thomas Eldon Allred
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#include <sstream>
#include <iostream>

typedef enum LOG_LEVEL{
	DISABLE = 8,//Turn off logging completely
	FATAL 	= 7,//Fatal or irrecoverable errors
	ERROR 	= 6,//Error Conditions
	WARNING	= 5,//Potential problems
	INFO	= 4,//Standard output
	DEBUG	= 3,//The general idea is the more function calls to get there the higher the debug level
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
#define GENERIC_LOG	(0x80000000)
#define ALL_LOG		(0xFFFFFFFF)
/**
 * LOG(level) macro allows us to not have to dump data into a stream if we aren't the log level is to low
 * Don't put non trivial function calls in the output stream or they wont be called when the logging isn't output
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

#define LOG(level)			LOG_STREAM(level,GENERIC_LOG)


class Log {
public:
	Log() {};
	virtual ~Log();
	std::stringstream & Get(LOG_LEVEL level = INFO);
public:
	static LOG_LEVEL & ReportingLevel();
	static unsigned int GetReportStreamBits();
	static void SetReportStreamBits( unsigned int s );
	static void SetTimeStampDisplay( bool ts );
	static void SetReportLevel( LOG_LEVEL level = INFO );
	static void SetLogFile( std::ostream * out );
	static std::ostream * GetOStream() { return outputStream; };
	static std::string ToString( LOG_LEVEL level );
protected:
	std::stringstream os;
protected:
	static std::string GetTime();
private:
	Log(const Log&);
	Log & operator = (const Log&);
private:
	LOG_LEVEL messageLevel;
private:
	static LOG_LEVEL logLevel;
	static unsigned int streamBits;
	static bool timestamp;
	static std::ostream * outputStream;

};

#endif /* LOGGING_H_ */
