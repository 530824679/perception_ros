#include "common/logging.h"

Logging::Logging() :
	last_relog_(false),
	min_level_(INFO),
	use_color_(true),
	cnt_errors_(0),
	cnt_warnings_(0) {
}

void Logging::SetLevel(LogLevel level){
	this->min_level_ = level;
};

void Logging::SetUseColor(bool use_color){
	this->use_color_ = use_color;
};

unsigned Logging::GetNumWarnings() const{
	return cnt_warnings_;
};

unsigned Logging::GetNumErrors() const{
	return cnt_errors_;
};

bool Logging::DoPrint(LogLevel level) const {
	// Log-level not fulfilled
	if(level > min_level_)
		return false;

	// It is not allowed to print in silence level
	if(level >= SILENCE)
		return false;

	return true;
}

void Logging::Log(LogLevel level, const char* fmt, ... ) {
	if(level == ERROR)
		cnt_errors_++;
	if(level == WARNING)
		cnt_warnings_++;

	if(! DoPrint(level))
		return;

	std::ostream & os  = level >= INFO ? std::cerr : std::cout;

	char text[1024];
	memset(text, 0, sizeof(char)*1023);

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, 1023, fmt, ap);
	va_end(ap);

	if(last_relog_) {
		last_relog_ = false;
		os << std::endl;
	}

	if(use_color_) {
		switch(level) {
			case ERROR:
				os << "\033[1;31m";
				break;
			case WARNING:
				os << "\033[1;33m";
				break;
			case DEBUG:
				os << "\033[1;30m";
				break;
			default:
				break;
		}
	}


	switch(level) {
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

	if(use_color_) {
		if( level==ERROR || level==WARNING || level==DEBUG )
		os << "\033[0;m";
	}

	os.flush();
}

void Logging::Log(LogLevel level, std::string &str) {
	Log(level, str.c_str());
}

void Logging::ReLog(LogLevel level, const char* fmt, ... ) {
	char text[1024];
	memset(text, 0, sizeof(char)*1023);

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, 1023, fmt, ap);
	va_end(ap);

	last_relog_ = false;
	Log(level, "\r%s", text);

	if( DoPrint(level) )
		last_relog_ = true;
}
