// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Ayush Dewan and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef G2O_EDGE_SE3_PRIOR_H_
#define G2O_EDGE_SE3_PRIOR_H_

#include "vertex_se3_append.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
namespace g2o {
  /**
   * \brief prior for an SE3 element
   *
   * Provides a prior for a 3d pose vertex. Again the measurement is represented by an
   * Isometry3D matrix.
   */
  class G2O_TYPES_SLAM3D_API EdgeICP : public BaseUnaryEdge<3, Vector3D, VertexSE3_Vector3D> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setMeasurement(const Vector3D& m){
      _measurement = m;
    }
    void computeError()
    {
      const VertexSE3_Vector3D* v1 = static_cast<const VertexSE3_Vector3D*>(_vertices[0]);

       _error =  (v1->estimate() * v1->getPosition() - _measurement) ;




    }

//   virtual void linearizeOplus();

/*
    // return the error estimate as a 3-vector

    // jacobian
    virtual void linearizeOplus();

        virtual bool setMeasurementData(const double* d){
      Eigen::Map<const Vector7d> v(d);
      _measurement = internal::fromVectorQT(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Eigen::Map<Vector7d> v(d);
      v = internal::toVectorQT(_measurement);
      return true;
    }

    virtual int measurementDimension() const {return 7;}

    virtual bool setMeasurementFromState() ;

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& ,
             OptimizableGraph::Vertex* ) {
      return 1.;
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
  protected:
    Isometry3D _inverseMeasurement;
    virtual bool resolveCaches();
    ParameterSE3Offset* _offsetParam;
    CacheSE3Offset* _cache;
    */








  };

}
#endif
