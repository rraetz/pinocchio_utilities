#pragma once

#ifdef LOGGING
#include "plog/Log.h"
#include "plog/Init.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#else
#include <iostream>
#include <sstream>
#endif


#ifndef LOGGING
// Define a null stream for macros when LOGGING is not defined
class NullStream : public std::ostream {
public:
    NullStream() : std::ostream(nullptr) {}
    template <typename T>
    NullStream& operator<<(const T&) { return *this; }
    NullStream& operator<<(std::ostream& (*)(std::ostream&)) { return *this; }
};

// Declare a static global instance of NullStream
inline NullStream nullStream;

// Define logging macros to use the null stream
#define LOG_VERBOSE nullStream
#define LOG_DEBUG   nullStream
#define LOG_INFO    nullStream
#define LOG_WARNING nullStream
#define LOG_ERROR   nullStream
#define LOG_FATAL   nullStream
#define LOG_NONE    nullStream

#endif


void init_logging()
{ 
    #ifdef LOGGING
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::verbose, &consoleAppender);  
    #endif
}
