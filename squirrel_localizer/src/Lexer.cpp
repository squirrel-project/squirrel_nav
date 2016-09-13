// Lexer.cpp --- 
// 
// Filename: Lexer.cpp
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:59:42 2015 (+0100)
// Version: 
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro
//   ROS Indigo
// 

// Code:

#include "squirrel_localizer/Lexer.h"

#include <cstdio>
#include <cstdlib>

using namespace std;

Lexer::Lexer(std::istream* is, std::ostream* os): yyFlexLexer(is,os){
    _currentToken=0;
  }
Lexer::~Lexer(){
    if (_currentToken)
      delete _currentToken;
  }

  bool Lexer::nextToken(){
    while (1) {
      int ret=yylex();
      if (ret==0)
	return false;
      if (_currentToken){
	delete _currentToken;
	_currentToken=0;
      }
      switch(ret){
      case LexerToken::Char:
	_currentToken=new LexerToken(YYText()[0]);
	break;
      case LexerToken::Int:
	_currentToken = new LexerToken(atoi (YYText()));
	break;
      case LexerToken::Double:
	_currentToken = new LexerToken(atof (YYText()));
	break;
      case LexerToken::ID: 
	_currentToken = new LexerToken(YYText());
	break;
      case LexerToken::String:
	_currentToken = new LexerToken(YYText()+1, YYLeng()-2);
	break;
      default: 
	_currentToken = 0;
	break;
      }
      if (ret>0)
	return true;
    }
    return true;
  }

  const LexerToken* Lexer::currentToken() const {
    return _currentToken;
  }

// 
// Lexer.cpp ends here
