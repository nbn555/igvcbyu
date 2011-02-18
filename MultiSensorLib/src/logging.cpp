/*
 * @file: logging.cpp
 * @date: Feb 17, 2011
 * @author: tallred3
 */

#include "logging.h"
#include "time.h"
#include <cstdio>
#include <iostream>

LOG_LEVEL Log::logLevel;
std::ostream * Log::outputStream = &(std::cerr);

std::string Log::GetTime() {
	time_t t = time(NULL);
	return std::string(asctime(gmtime(&t)));
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

	this->os << "- " << Log::GetTime();
	this->os << " " << Log::ToString(level) << ": ";

	this->os << std::string(level > DEBUG ? 1 : level - DEBUG, '\t' );
	messageLevel = level;
	return os;
}

LOG_LEVEL & Log::ReportingLevel() {	return logLevel; }

void Log::SetReportLevel( LOG_LEVEL level ) { logLevel = level; }

Log::~Log() {

	if (messageLevel >= Log::ReportingLevel())
	{
		(*Log::GetOStream()) << os.str();
		Log::GetOStream()->flush();
	}
}

void Log::SetLogFile( std::ostream * out ) {
	Log::outputStream = out;
}
