// RuntimeError.h --- 
// 
// Filename: RuntimeError.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:11:01 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//    ROS Hydro
//    ROS Indigo
// 

// Code:

#ifndef SQUIRREL_LOCALIZER_RUNTIME_ERROR_H_
#define SQUIRREL_LOCALIZER_RUNTIME_ERROR_H_

#include <exception>
#include <string>

/**
 * \brief a run time error exception
 */
class RuntimeError : public std::exception
{
  public:
    /**
     * constructor which allows to give a error message
     * with printf like syntax
     */
    explicit RuntimeError(const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
    virtual ~RuntimeError() throw();
    virtual const char* what() const throw() {return _errorMsg.c_str();}

  protected:
    std::string _errorMsg;
};

#endif /* SQUIRREL_LOCALIZER_RUNTIME_ERROR_H_ */

//
// RuntimeError.h ends here
