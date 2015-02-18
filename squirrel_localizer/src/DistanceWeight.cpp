// DistanceWeight.cpp --- 
// 
// Filename: DistanceWeight.cpp
// Description: Weight distance functions
// Author: Joerg Roewenkaemper
// Maintainer: bonairdi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:09:56 2015 (+0100)
// Version: 
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

#include "squirrel_localizer/DistanceWeight.h"

#include <cmath>

using namespace std;

DistanceWeight::~DistanceWeight()
{}

double DistanceWeight::weight(double dist) const
{
    return 0.0 * dist;
}


LevyDistance::LevyDistance(double gamma)
    :   m_gamma(gamma)
{
    m_limit = m_gamma / 3.0;
    m_maxvalue = sqrt(m_gamma / (2 * M_PI)) *
                 exp(-m_gamma/(2*m_limit)) /
                 (pow(m_limit, 1.5));
}

double LevyDistance::weight(double dist) const
{
    if(dist < m_limit) {
        return m_maxvalue;
    } else {
        return sqrt(m_gamma / (2 * M_PI)) *
               exp(-m_gamma/(2*dist)) /
               (pow(dist, 1.5));
    }

}

GaussianDistance::GaussianDistance(double sigma)
    :   m_sigma(sigma)
{
    m_prefactor = 1.0 / (m_sigma * sqrt(2 * M_PI));
    m_sigma_square = m_sigma * m_sigma * 2;
}

double GaussianDistance::weight(double dist) const
{
    return m_prefactor * exp(- (dist*dist) / m_sigma_square);
}

double ExponentialDistance::weight(double dist) const
{
    return exp(-dist);
}

// 
// DistanceWeight.cpp ends here
