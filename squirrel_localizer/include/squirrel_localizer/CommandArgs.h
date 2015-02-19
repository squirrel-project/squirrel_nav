// CommandArgs.h --- 
// 
// Filename: CommandArgs.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:19:42 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro 
//   ROS Indigo
// 

// Commentary:
//
//   /**************************************************************************
//    File: commandArgs.h
//    Copyright (c) 2009 Rainer Kümmerle <rk@raikue.net>
// 
//    This program is free software: you can redistribute it and/or modify it
//    under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or (at
//    your option) any later version.
// 
//    this program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
// 
//    You should have received a copy of the GNU Lesser General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//   ****************************************************************************/
//   

// Code:

#ifndef SQUIRREL_LOCALIZER_COMMAND_ARGS_H_
#define SQUIRREL_LOCALIZER_COMMAND_ARGS_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

namespace AISNavigation {

class CommandArgs
{
 public:
  struct CommandArgument
  {
    std::string name;
    std::string description;
    int type;
    void* data;
    bool parsed;
    bool optional;
    CommandArgument() : name(""), description(""), type(0), data(0), parsed(false), optional(false)
    {}
  };
 public:
  CommandArgs();
  virtual ~CommandArgs();

  bool parseArgs(int argc, char** argv, bool exitOnError = true);

  void param(const std::string& name, bool& p, bool defValue, const std::string& desc);

  void param(const std::string& name, int& p, int defValue, const std::string& desc);
  /** add a float parameter */
  void param(const std::string& name, float& p, float defValue, const std::string& desc);
  /** add a float parameter */
  void param(const std::string& name, double& p, double defValue, const std::string& desc);
  /** add a string parameter */
  void param(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc);
  /** add a param wich is specified as a plain argument */
  void paramLeftOver(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc, bool optional = false);

  /**
   * print the value of all params to an ostream
   */
  void printParams(std::ostream& os);

  //! return the banner string
  const std::string& getBanner() const { return _banner; }
  void setBanner(const std::string& banner);

  /**
   * print the help
   */
  void printHelp(std::ostream& os);

 protected:
  std::vector<CommandArgument> _args;
  std::vector<CommandArgument> _leftOvers;
  std::vector<CommandArgument> _leftOversOptional;
  std::string _banner;
  std::string _progName;

  const char* type2str(int t) const;
  void str2arg(const std::string& input, CommandArgument& ca) const;
  std::string arg2str(const CommandArgument& ca) const;

  /**
   * convert a string into an other type.
   */
  template<typename T>
  bool convertString(const std::string& s, T& x) const
  {
    std::istringstream i(s);
    bool status = (i >> x);
    return status;
  }

  std::string trim(const std::string& s) const;

  /** Helper class to sort pair based on first elem */
  template<class T1, class T2, class Pred = std::less<T1> >
  struct CmpPairFirst {
    bool operator()(const std::pair<T1,T2>& left, const std::pair<T1,T2>& right) {
      return Pred()(left.first, right.first);
    }
  };

};

} // end namespace

#endif /* SQUIRREL_LOCALIZER_COMMAND_ARGS_H_ */

//
// CommandArgs.h ends here
