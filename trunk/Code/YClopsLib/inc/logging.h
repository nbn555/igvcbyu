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
	DEBUG	= 3,//High level debugging output
	DEBUG2	= 2,
	DEBUG3	= 1,//Raw sensor data kind of stuff
	DEBUG4	= 0	//step by step instructions of program flow
} LOG_LEVEL;

#define LOG(level) \
(level < Log::ReportingLevel()) ? : Log().Get(level)

class Log {
public:
	Log() {};
	virtual ~Log();
	std::stringstream & Get(LOG_LEVEL level = INFO );
public:
	static LOG_LEVEL & ReportingLevel();
	static void SetReportLevel( LOG_LEVEL level = INFO );
	static void SetLogFile( std::ostream * out );
	static std::ostream * GetOStream() { return outputStream; };
protected:
	std::stringstream os;
protected:
	static std::string GetTime();
	static std::string ToString( LOG_LEVEL level );
private:
	Log(const Log&);
	Log & operator = (const Log&);
private:
	LOG_LEVEL messageLevel;
private:
	static LOG_LEVEL logLevel;
	static std::ostream * outputStream;

};

#endif /* LOGGING_H_ */