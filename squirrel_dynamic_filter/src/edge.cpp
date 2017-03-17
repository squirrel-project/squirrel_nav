// Copyright (c) 2016-2017, Ayush Dewan and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge.h"
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include <iostream>
#include "g2o/core/factory.h"
#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

  using namespace std;
  using namespace Eigen;
  using namespace g2o;
  using namespace g2o::internal;

  G2O_REGISTER_TYPE(EDGE_SE3:BINARY, Edge);

  Edge::Edge() : BaseBinaryEdge<6, Isometry3D,VertexSE3_Vector3D,VertexSE3_Vector3D>() {

    //cerr << "dsdsE" << endl;

    information().setIdentity();
  }
  bool Edge::read(std::istream& is) {
    Vector7d meas;
    for (int i=0; i<7; i++)
      is >> meas[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4D::MapType(meas.data()+3).normalize();
    setMeasurement(fromVectorQT(meas));

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
  ;  //  we overwrite the information matrix with the Identity
      information().setIdentity();
    }
    return true;
  }

  bool Edge::write(std::ostream& os) const {
    Vector7d meas=toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }

  void Edge::computeError() {
    VertexSE3_Vector3D *from = static_cast<VertexSE3_Vector3D*>(_vertices[0]);
    VertexSE3_Vector3D *to   = static_cast<VertexSE3_Vector3D*>(_vertices[1]);
    Isometry3D delta=_inverseMeasurement * (from->estimate().inverse() * to->estimate());
    _error=toVectorMQT(delta);
    _error = _error;
   // fprintf(stderr,"inside error");

  }
  bool Edge::setMeasurementFromState(){
    VertexSE3_Vector3D *from = static_cast<VertexSE3_Vector3D*>(_vertices[0]);
    VertexSE3_Vector3D *to   = static_cast<VertexSE3_Vector3D*>(_vertices[1]);
    Isometry3D delta = from->estimate().inverse() * to->estimate();
    setMeasurement(delta);
    return true;
  }

  void Edge::linearizeOplus(){

    // BaseBinaryEdge<6, Isometry3D, VertexSE3, VertexSE3>::linearizeOplus();
    // return;

    VertexSE3_Vector3D *from = static_cast<VertexSE3_Vector3D*>(_vertices[0]);
    VertexSE3_Vector3D *to   = static_cast<VertexSE3_Vector3D*>(_vertices[1]);
    Isometry3D E;
    const Isometry3D& Xi=from->estimate();
    const Isometry3D& Xj=to->estimate();
    const Isometry3D& Z=_measurement;
    computeEdgeSE3Gradient(E, _jacobianOplusXi , _jacobianOplusXj, Z, Xi, Xj);
  }

  void Edge::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3_Vector3D *from = static_cast<VertexSE3_Vector3D*>(_vertices[0]);
    VertexSE3_Vector3D *to   = static_cast<VertexSE3_Vector3D*>(_vertices[1]);

    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * _measurement);
    } else
      from->setEstimate(to->estimate() * _measurement.inverse());
    //cerr << "IE" << endl;
  }

  EdgeSE3WriteGnuplotAction::EdgeSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(Edge).name()){}

  HyperGraphElementAction* EdgeSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    Edge* e =  static_cast<Edge*>(element);
    VertexSE3_Vector3D* fromEdge = static_cast<VertexSE3_Vector3D*>(e->vertices()[0]);
    VertexSE3_Vector3D* toEdge   = static_cast<VertexSE3_Vector3D*>(e->vertices()[1]);
    Vector6d fromV, toV;
    fromV=toVectorMQT(fromEdge->estimate());
    toV=toVectorMQT(toEdge->estimate());
    for (int i=0; i<6; i++){
      *(params->os) << fromV[i] << " ";
    }
    for (int i=0; i<6; i++){
      *(params->os) << toV[i] << " ";
    }
    *(params->os) << std::endl;
    return this;
  }


