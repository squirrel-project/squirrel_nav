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
}
