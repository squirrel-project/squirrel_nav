// LexerToken.h --- 
// 
// Filename: LexerToken.h
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

#ifndef SQUIRREL_LOCALIZER_LEXERTOKEN_H_
#define SQUIRREL_LOCALIZER_LEXERTOKEN_H_

#include <string>

struct LexerTokenValue{
	char	    charValue;
	int	    intValue;
	double	    doubleValue;
	std::string  strValue;
};

struct LexerToken {
  enum Type{Invalid=-1, Char=1, Int=2, Double=3, String=4, ID=5};
	Type type() const {return _type;}
	const LexerTokenValue& value() const {return _value;}
	LexerToken (char c){
		_type=Char;
		_value.charValue=c;
	};
	LexerToken (const char* c){
		_type=ID;	
		_value.strValue=std::string(c);
	};
	LexerToken (const char* c1, int lenght){
		_type=String;
		_value.strValue=std::string(c1,c1+lenght);
	};
	LexerToken (int i){
		_type=Int;
		_value.intValue=i;
	};
	LexerToken (double d){
		_type=Double;
		_value.doubleValue=d;
	};

protected:		
	LexerTokenValue _value;
	Type _type;
};

#endif  /* SQUIRREL_LOCALIZER_LEXERTOKEN_H_ */

//
// LexerToken.h ends here
