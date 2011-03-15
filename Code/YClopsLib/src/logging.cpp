/*
 * @file: logging.cpp
 * @date: Feb 17, 2011
 * @author: tallred3
 */

#include "logging.h"
#include "time.h"
#include <cstdio>
#include <iostream>
#include <exception>

LOG_LEVEL Log::logLevel = INFO;
std::ostream * Log::outputStream = &(std::cerr);
unsigned int Log::streamBits = ~ALL_LOG;
bool Log::timestamp = true;

using namespace std;

std::string Log::GetTime() {
	time_t t = time(NULL);
	return std::string(asctime(gmtime(&t))); //return a formatted string of the timestamp
}

std::string Log::ToString( LOG_LEVEL level ) {
	switch(level){
	case FATAL:
		return std::string("FATAL");
		break;
	case ERROR:
		return std::string("ERROR");
		break;
	case WARNING:
		return std::string("WARNING");
		break;
	case INFO:
		return std::string("INFO");
		break;
	case DEBUG:
		return std::string("DEBUG");
		break;
	case DEBUG2:
		return std::string("DEBUG2");
		break;
	case DEBUG3:
		return std::string("DEBUG3");
		break;
	case DEBUG4:
		return std::string("DEBUG4");
		break;
	default:
		return std::string("ERROR IN LOGGING UTILITY");
		break;
	}
}

std::stringstream& Log::Get(LOG_LEVEL level) {

	if(Log::timestamp) {
		this->os << "- " << Log::GetTime();
	}

	this->os << " " << Log::ToString(level) << ": ";
	this->os << std::string(level > DEBUG ? 1 :  DEBUG - level, '\t' ); //Set increasing number of tabs for higher debug levels
	messageLevel = level;
	return os;
}

LOG_LEVEL & Log::ReportingLevel() {	return Log::logLevel; }

unsigned int Log::GetReportStreamBits() { return Log::streamBits; }

void Log::SetReportStreamBits( unsigned int s ) { Log::streamBits = ~s; }

void Log::SetTimeStampDisplay( bool ts ) { Log::timestamp = ts; }

void Log::SetReportLevel( LOG_LEVEL level ) { logLevel = level; }

Log::~Log() {

	if (messageLevel >= Log::ReportingLevel())
	{
		(*Log::GetOStream()) << os.str();
		Log::GetOStream()->flush();
	}

	if( FATAL == messageLevel ) {
		throw new exception();
	}
}

void Log::SetLogFile( std::ostream * out ) {
	Log::outputStream = out;
}