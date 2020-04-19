#ifndef EXCEPTIONCLASS_H
#define EXCEPTIONCLASS_H
#include <exception>
#include <string>
#include <string.h>


class ExceptionClass : public std::exception
{
protected:
    std::string error_message;

public:
    ExceptionClass(const std::string& error_message);

//    ~Exception() throw(){}

    virtual const char* what() const throw()
    {
        return error_message.c_str();
    }
};


#endif // EXCEPTIONCLASS_H
