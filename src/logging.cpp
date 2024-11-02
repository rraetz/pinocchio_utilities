#include "plog/Log.h"
#include "plog/Init.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Appenders/ColorConsoleAppender.h"


void init_logging()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::verbose, &consoleAppender);
}