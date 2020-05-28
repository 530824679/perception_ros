//
/******************************************************************************/
/*!
File name: logging.h

Description:
This file define class of the logging to realize printing information

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai
Example: logger.log(DEBUG, "[%s]: MSE=%.8f in %d-th RLS trial\n", __func__, mse, trialCount);

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef PERCEPTION_ROS_LOGGING_H
#define PERCEPTION_ROS_LOGGING_H

// system include
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ostream>

// local include
#include "types.h"

class Logging{
public:

	Logging();
	void SetLevel(LogLevel level);
	void SetUseColor(bool use_color);
	unsigned GetNumWarnings() const;
	unsigned GetNumErrors() const;

	void Log(LogLevel level, const char* fmt, ... );
	void Log(LogLevel level, std::string& str);
	void ReLog(LogLevel level, const char* fmt, ... );

private:
	bool DoPrint(LogLevel level) const;

private:
	bool last_relog_;
	bool use_color_;
	LogLevel min_level_;

	unsigned cnt_errors_;
	unsigned cnt_warnings_;
};


#endif //PERCEPTION_ROS_LOGGING_H
