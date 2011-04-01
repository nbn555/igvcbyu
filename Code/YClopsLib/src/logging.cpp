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

//#define LOG_COLORS

LOG_LEVEL Log::logLevel = INFO;
std::ostream * Log::outputStream = &(std::cerr);
unsigned int Log::streamBits = ~ALL_LOG;
bool Log::timestamp = true;

#ifdef NVIEW
NView * Log::view;
#endif

using namespace std;

#ifdef LOG_COLORS
#define LOG_BLACK			"\e[0;30m"
#define LOG_BLUE			"\e[0;34m"
#define LOG_GREEN			"\e[0;32m"
#define LOG_CYAN	        "\e[0;36m"
#define LOG_RED				"\e[0;31m"
#define LOG_PURPLE			"\e[0;35m"
#define LOG_BROWN			"\e[0;33m"
#define LOG_GRAY			"\e[0;37m"
#define LOG_DARK_GRAY		"\e[1;30m"
#define LOG_LIGHT_BLUE		"\e[1;34m"
#define LOG_LIGHT_GREEN		"\e[1;32m"
#define LOG_LIGHT_CYAN		"\e[1;36m"
#define LOG_LIGHT_RED		"\e[1;31m"
#define LOG_LIGHT_PURPLE	"\e[1;35m"
#define LOG_YELLOW			"\e[1;33m"
#define LOG_WHITE			"\e[1;37m"
#define LOG_END_COLOR		"\e[m"
#endif

std::string Log::GetTime() {
	time_t t = time(NULL);
	return std::string(asctime(gmtime(&t))); //return a formatted string of the timestamp
}

std::string Log::ToString( LOG_LEVEL level ) {
	switch(level){
	case OUT:
		return std::string("OUT");
		break;
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

#ifdef LOG_COLORS
	switch(level) {
	case OUT:
		break;
	case FATAL:
		this->os << LOG_RED;
		break;
	case ERROR:
		break;
	case WARNING:
		break;
	case INFO:
		this->os << LOG_WHITE;
		break;
	case DEBUG:
		this->os << LOG_DARK_GRAY;
		break;
	case DEBUG2:
		this->os << LOG_GRAY;
		break;
	case DEBUG3:
		this->os << LOG_BLUE;
		break;
	case DEBUG4:
		this->os << LOG_LIGHT_BLUE;
		break;
	default:
		break;
	}
#endif

	this->os << " " << Log::ToString(level) << ": ";
	//this->os << std::string(level > DEBUG ? 1 :  DEBUG - level, '\t' ); //Set increasing number of tabs for higher debug levels
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
#ifndef NVIEW
		(*Log::GetOStream()) << os.str();
#endif
#ifdef LOG_COLORS
		(*Log::GetOStream()) << LOG_END_COLOR;
#endif
#ifndef NVIEW
		Log::GetOStream()->flush();
#else
		Log::view->log(os.str());

#endif

	}

	if( FATAL == messageLevel ) {
		throw new exception();
	}
}

void Log::SetLogFile( std::ostream * out ) {
	Log::outputStream = out;
}
