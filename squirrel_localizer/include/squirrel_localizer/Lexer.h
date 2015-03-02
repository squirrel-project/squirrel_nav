// Lexer.h --- 
// 
// Filename: Lexer.h
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

#ifndef SQUIRREL_LOCALIZER_LEXER_H_
#define SQUIRREL_LOCALIZER_LEXER_H_

#include <iostream>

#include <FlexLexer.h>

#include "squirrel_localizer/LexerToken.h"

struct Lexer: public yyFlexLexer{
  Lexer(std::istream* is=&std::cin, std::ostream* os=&std::cout);
  ~Lexer();
  bool nextToken();
  const LexerToken* currentToken() const ;
protected:
  LexerToken* _currentToken;
};

#endif  /* SQUIRREL_LOCALIZER_LEXER_H_ */

//
// Lexer.h ends here
