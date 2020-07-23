#include "exceptionclass.h"
#include <string.h>
#
ExceptionClass::ExceptionClass(const std::string& error_message, WarningLevelEnum warning_level)  : error_message(error_message),
warning_level(warning_level)
{}


