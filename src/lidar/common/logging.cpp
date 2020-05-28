#include "logging.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <ostream>

logging::logging() :
	lastrelog(false),
	minLevel(INFO),
	useColor(true),
	cntErrors(0),
	cntWarnings(0)
{
}


bool logging::doPrint(loglevel level) const {

	// Log-level not fulfilled
	if( level < minLevel )
		return false;

	// It is not allowed to print in silence level
	if( level >= SILENCE )
		return false;

	return true;
}


void logging::log(loglevel level, const char* fmt, ... ) {

	// Save some statistics
	if(level == ERROR)
		cntErrors++;
	if(level == WARNING)
		cntWarnings++;


	// Check if level suffices...
	if( ! doPrint(level) )
		return;

	// Default output of log message
	std::ostream & os  = level >= INFO ? std::cerr : std::cout;

	char text[1024];
	memset(text, 0, sizeof(char)*1023);

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, 1023, fmt, ap);
	va_end(ap);


	// Make a new line if last cmd was a relog command
	if(lastrelog)
	{
		lastrelog=false;
		os << std::endl;
	}

	// Switch color
	if( useColor )
	{
		switch( level )
		{
			case ERROR:
				os << "\033[1;31m";
				break;
			case WARNING:
				os << "\033[1;33m";
				break;
			case GOODNEWS:
				os << "\033[1;32m";
				break;
			case DEBUG:
				os << "\033[1;30m";
				break;
			default:
				break;
		}
	}


	switch( level )
	{
		case ERROR:
			os << "ERROR: ";
			break;
		case WARNING:
			os << "WARNING: ";
			break;
		default:
			break;
	}

	os << text;

	if( useColor ) {
		if( level==ERROR || level==WARNING || level==GOODNEWS || level==DEBUG )
		os << "\033[0;m";
	}

	os.flush();
}

void logging::log( loglevel level, std::string &str) {
	log(level, str.c_str());
}

void logging::relog(loglevel level, const char* fmt, ... ) {
	char text[1024];
	memset(text, 0, sizeof(char)*1023);

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, 1023, fmt, ap);
	va_end(ap);

	lastrelog = false;
	log(level, "\r%s", text);

	// Check if level suffices...
	if( doPrint(level) )
		lastrelog = true;
}
