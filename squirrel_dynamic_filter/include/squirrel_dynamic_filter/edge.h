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

#include "g2o/core/eigen_types.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "vertex_se3_append.h"

using namespace g2o;
using namespace g2o::internal;

  class G2O_TYPES_SLAM3D_API Edge : public BaseBinaryEdge<6, Isometry3D, VertexSE3_Vector3D, VertexSE3_Vector3D> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Edge();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Isometry3D& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector7d> v(d);
        setMeasurement(::internal::fromVectorQT(v));
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector7d> v(d);
        v = ::internal::toVectorQT(_measurement);
        return true;
      }

      void linearizeOplus();

      virtual int measurementDimension() const {return 7;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      Isometry3D _inverseMeasurement;
  };

  /**
   * \brief Output the pose-pose constraint to Gnuplot data file
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };







