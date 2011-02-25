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
	DEBUG	= 3,//High level debugging output at the main.cpp kind of level
	DEBUG2	= 2,//At the reactive nav interface / AI algorithm level
	DEBUG3	= 1,//step by step instructions of program flow at the sensor level
	DEBUG4	= 0	//low level stuff
} LOG_LEVEL;

/**
 * LOG(level) macro allows us to not have to dump data into a stream if we aren't the log level is to low
 * Don't put non trivial function calls in the output stream or they wont be called when the logging isn't output
 */
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
