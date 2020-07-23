#ifndef EXCEPTIONCLASS_H
#define EXCEPTIONCLASS_H
#include <exception>
#include <string>
#include <string.h>
#include <iostream>
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
        if( warning_level == WarningLevelEnum::Error)
            std::cout<<error_message.c_str();

    }
};


#endif // EXCEPTIONCLASS_H
