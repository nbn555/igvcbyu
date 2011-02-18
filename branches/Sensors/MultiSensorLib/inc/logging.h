/*
 * logging.h
 *
 *  Created on: Feb 17, 2011
 *      Author: tallred3
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#include <sstream>
#include <iostream>

typedef enum LOG_LEVEL{
	FATAL 	= 7,
	ERROR 	= 6,
	WARNING	= 5,
	INFO	= 4,
	DEBUG	= 3,
	DEBUG2	= 2,
	DEBUG3	= 1,
	DEBUG4	= 0
} LOG_LEVEL;

#define LOG(level) \
if (level < Log::ReportingLevel()) ; \
else Log().Get(level)

class Log {
public:
	Log() {};
	virtual ~Log();
	std::stringstream & Get(LOG_LEVEL level = INFO );
public:
	static LOG_LEVEL & ReportingLevel();
	static void SetReportLevel( LOG_LEVEL level = INFO );
	static void SetLogFile( std::ostream * out );
protected:
	std::stringstream os;
protected:
	static std::string GetTime();
	static std::string ToString( LOG_LEVEL level );
	static std::ostream * GetOStream() { return outputStream; };
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
