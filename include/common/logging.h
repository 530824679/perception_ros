//
// Created by chenwei on 20-5-28.
//

#ifndef PERCEPTION_ROS_LOGGING_H
#define PERCEPTION_ROS_LOGGING_H

// system include
#include <string>


// logger.log(DEBUG, "[%s]: MSE=%.8f in %d-th RLS trial\n", __func__, mse, trialCount);

enum LogLevel{
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Logging{

	private:

		bool lastrelog;
		loglevel minLevel;
		bool useColor;

		unsigned cntErrors;
		unsigned cntWarnings;

    public:

		// Create a new logging
		logging();

		// Set new log level
		void setLevel(loglevel level)	{ minLevel = level; };

		// Use color or not?
		void setUsecolor(bool useColor)	{ this->useColor = useColor; };

		// Get number of warnings
		unsigned getNoWarnings() const	{ return cntWarnings; };

		// Get number of errors
		unsigned getNoErrors() const	{ return cntErrors; };

		// Log a new message in well-known printf manner
		void log(loglevel level, const char* fmt, ... );

		// Log a new message
		void log(loglevel level, std::string& str);

		// Like log(), but overwrite the last line
		void relog(loglevel level, const char* fmt, ... );

	private:

		// Check if we print in this level
		bool doPrint(loglevel level) const;
};


#endif //PERCEPTION_ROS_LOGGING_H
