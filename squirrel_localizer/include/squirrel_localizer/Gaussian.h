// Gaussian.h --- 
// 
// Filename: Gaussian.h
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

#ifndef SQUIRREL_LOCALIZER_GAUSSIAN_H_
#define SQUIRREL_LOCALIZER_GAUSSIAN_H_

#include "squirrel_localizer/Vector_n,h"
#include "squirrel_localuzer/Matrix_n,h"

template <int N, typename Base>
  struct _Gaussian {
    _Gaussian(int s);
    _Gaussian(const Vector<N, Base>& v_, const Matrix<N,N,Base> m_, bool isCovForm);
    const Vector<N, Base> mean() const;
    const Matrix<N, N, Base> covariance() const;
    const Vector<N, Base> infoVector() const;
    const Matrix<N, N, Base> information() const;
    
    void setMean(const Vector<N, Base>& mean_);
    void setCovariance(const Matrix<N, N, Base>& cov_);
    void setInformationVector(const Vector<N, Base>& iv_);
    void setInformationMatrix(const Matrix<N, N, Base>& cov_);

    _Gaussian<N, Base> operator + (const _Gaussian <N, Base>& g);
    
  protected:
    updateInfoForm();
    updateCovform();
    mutable _Matrix<N,N,Base> _covariance;
    mutable _Matrix<N,N,Base> _information;
    mutable _Vector<N,Base> _mean;
    mutable _Vector<N,Base> _informationVector;
    mutable bool _infoFormUpdated;
    mutable bool _covarianceFormUpdated;
  };

template <int N, int M, typename Base>
  _Gaussian<N,Base> operator * (_Matrix<M,N,Base> m, _Gaussian<M,Base> g){
}

#endif  /* SQUIRREL_LOCALIZER_GAUSSIAN_H_ */

// 
// Gaussian.h ends here
