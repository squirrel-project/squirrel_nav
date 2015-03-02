// BasicMotion.cpp --- 
// 
// Filename: BasicMotion.cpp
// Description: Basic motion model for particle filter
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:39:23 2015 (+0100)
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

// Code:


#include "squirrel_localizer/BasicMotion.h"

#include <iostream>

using namespace std;


void BasicMotion::update_motion(const Transformation3 &motion)
{
    m_cummulative_motion = motion;
    Vector6 motion6d = m_cummulative_motion.toVector();
    m_linear_motion += m_cummulative_motion.translation().norm();
    m_angular_motion += fabs(motion6d[5]);
    //std::cout << motion << " | " << m_linear_motion << " " << m_angular_motion << std::endl;

}

void BasicMotion::reset()
{
    m_linear_motion = 0.0;
    m_angular_motion = 0.0;
    m_cummulative_motion = Transformation3();
}

double BasicMotion::linear_motion() const
{
    return m_linear_motion;
}

double BasicMotion::angular_motion() const
{
    return m_angular_motion;
}

Transformation3 BasicMotion::cummulative_motion() const
{
    return m_cummulative_motion;
}

// 
// BasicMotion.cpp ends here
