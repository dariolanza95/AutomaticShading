#ifndef EXCEPTIONCLASS_H
#define EXCEPTIONCLASS_H
#include <exception>
#include <string>
#include <string.h>

enum class WarningLevelEnum{Error,Warning,Info};
class ExceptionClass : public std::exception
{
protected:
    std::string error_message;
    WarningLevelEnum warning_level;
public:
    ExceptionClass(const std::string& error_message,WarningLevelEnum warning_level = WarningLevelEnum::Info);

//    ~Exception() throw(){}

    virtual const char* what() const throw()
    {
        return error_message.c_str();
    }
};


#endif // EXCEPTIONCLASS_H
