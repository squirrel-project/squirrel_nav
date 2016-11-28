// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_unary.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include <iostream>
#include "g2o/core/factory.h"
using namespace Eigen;
namespace g2o {
  using namespace std;

  G2O_REGISTER_TYPE(EDGE_ICP:UNARY, EdgeICP);
  // point to camera projection, monocular
  EdgeICP::EdgeICP() : BaseUnaryEdge<3, Vector3D, VertexSE3_Vector3D>() {
  }

    bool EdgeICP::read(std::istream& is)
    {
      Vector3D p;
      is >> p[0] >> p[1] >> p[2];
      setMeasurement(p);
      for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
          is >> information()(i, j);
          if (i != j)
            information()(j, i) = information()(i, j);
        }
      return true;
    }


  bool EdgeICP::write(std::ostream& os) const
    {

      os << measurement()[0] << " " << measurement()[1]  << " " << measurement()[2] ;
      for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j)
          os << " " << information()(i, j);
      return os.good();
    }

  /*
void EdgeSE3Prior::linearizeOplus(){
    VertexSE3_Vector3D *v = static_cast<VertexSE3_Vector3D*>(_vertices[0]);
    _jacobianOplusXi.setZero();

         Isometry3D::ConstLinearPart Ri = extractRotation(v->estimate());

      _jacobianOplusXi.template block<3,3>(0,0)=Ri;

      Matrix3D s;

      Vector3D p=v->getPosition();

     s(0,0)=0;       s(0,1)=-2*p(2);   s(0,2)=2*p(1);

     s(1,0)=2*p(2); s(1,1)=0;        s(1,2)=-2*p(0);


    s(2,0)=-2*p(1);  s(2,1)=2*p(0);  s(2,2)=0;

    _jacobianOplusXi.template block<3,3>(0,3)=s;


    // fprintf(stderr,"%f,%f,%f,%f,\n",_jacobianOplusXi(0,3),_jacobianOplusXi(1,3),_jacobianOplusXi(2,3),_jacobianOplusXi(1,4));



}
*/
/*
  bool EdgeSE3Prior::resolveCaches(){
    assert(_offsetParam);
    ParameterVector pv(1);
    pv[0]=_offsetParam;
    resolveCache(_cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    return _cache != 0;
  }



  bool EdgeSE3Prior::read(std::istream& is) {
    int pid;
    is >> pid;
    if (!setParameterId(0, pid))
      return false;
    // measured keypoint
    Vector7d meas;
    for (int i=0; i<7; i++) is >> meas[i];
    setMeasurement(internal::fromVectorQT(meas));
    // don't need this if we don't use it in error calculation (???)
    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
    }
    return true;
  }

  bool EdgeSE3Prior::write(std::ostream& os) const {
    os << _offsetParam->id() <<  " ";
    Vector7d meas = internal::toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Prior::computeError() {
    Isometry3D delta=_inverseMeasurement * _cache->n2w();
    _error = internal::toVectorMQT(delta);
  }


  bool EdgeSE3Prior::setMeasurementFromState(){
    setMeasurement(_cache->n2w());
    return true;
  }

  void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);
    assert(v && "Vertex for the Prior edge is not set");

    Isometry3D newEstimate = _offsetParam->offset().inverse() * measurement();
    if (_information.block<3,3>(0,0).array().abs().sum() == 0){ // do not set translation, as that part of the information is all zero
      newEstimate.translation()=v->estimate().translation();
    }
    if (_information.block<3,3>(3,3).array().abs().sum() == 0){ // do not set rotation, as that part of the information is all zero
      newEstimate.matrix().block<3,3>(0,0) = internal::extractRotation(v->estimate());
    }
    v->setEstimate(newEstimate);
  }

    */
}
