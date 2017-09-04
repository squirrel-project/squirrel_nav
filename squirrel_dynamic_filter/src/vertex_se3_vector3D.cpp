
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
#include "vertex_se3_append.h"
#include "g2o/core/factory.h"
//#ifdef G2O_HAVE_OPENGL
//#include "g2o/stuff/opengl_wrapper.h"
//#include "g2o/stuff/opengl_primitives.h"
//#endif

#include <iostream>
#include "g2o/core/cache.h"

using namespace Eigen;
using namespace g2o;
using namespace g2o::internal;

  G2O_REGISTER_TYPE(VERTEX_SE3:ICP,VertexSE3_Vector3D);

  VertexSE3_Vector3D::VertexSE3_Vector3D() :
    BaseVertex<6, Isometry3D>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }



  void VertexSE3_Vector3D::setPosition(Vector3D position_input)
  {
    position=position_input;
  }

  Vector3D VertexSE3_Vector3D::getPosition() const
  {

    return position;


  }





  bool VertexSE3_Vector3D::read(std::istream& is)
  {
    Vector7d est;
    Vector3D pos;
    std::vector<double>input(10);
    for (int i=0; i<10; i++)
    {
      is >> input[i];

       }
     for (int i=0; i<7; i++)
    {
      est[i]=input[i];

       }
   for (int i=7; i<10; i++)
    {
      pos[i-7]=input[i];

       }



    setEstimate(fromVectorQT(est));

    setPosition(pos);







    return true;
  }

  bool VertexSE3_Vector3D::write(std::ostream& os) const
  {
    Vector7d est=toVectorQT(_estimate);
    for (int i=0; i<7; i++)
      os << est[i] << " ";
    Vector3D pos=position;
     for (int i=0; i<3; i++)
      os << pos[i] << " ";

    return os.good();
  }

  VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE3_Vector3D).name()){}

  HyperGraphElementAction* VertexSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return 0;
    }

    VertexSE3_Vector3D* v =  static_cast<VertexSE3_Vector3D*>(element);
    Vector6d est=toVectorMQT(v->estimate());
    for (int i=0; i<6; i++)
      *(params->os) << est[i] << " ";
        *(params->os) << std::endl;
    return this;
  }
/*
#ifdef G2O_HAVE_OPENGL
  void drawTriangle(float xSize, float ySize){
    Vector3F p[3];
    glBegin(GL_TRIANGLES);
    p[0] << 0., 0., 0.;
    p[1] << -xSize, ySize, 0.;
    p[2] << -xSize, -ySize, 0.;
    for (int i = 1; i < 2; ++i) {
      Vector3F normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
      glNormal3f(normal.x(), normal.y(), normal.z());
      glVertex3f(p[0].x(), p[0].y(), p[0].z());
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
      glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
    }
    glEnd();
  }
  VertexSE3DrawAction::VertexSE3DrawAction(): DrawAction(typeid(VertexSE3_Vector3D).name()){
    _cacheDrawActions = 0;
  }

  bool VertexSE3DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_X", .2f);
      _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_Y", .05f);
    } else {
      _triangleX = 0;
      _triangleY = 0;
    }
    return true;
  }
  HyperGraphElementAction* VertexSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element,
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params_);

    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    VertexSE3_Vector3D* that = static_cast<VertexSE3_Vector3D*>(element);

    glColor3f(POSE_VERTEX_COLOR);
    glPushMatrix();
    glMultMatrixd(that->estimate().matrix().data());
    opengl::drawArrow2D(_triangleX->value(), _triangleY->value(), _triangleX->value()*.3f);
    drawCache(that->cacheContainer(), params_);
    drawUserData(that->userData(), params_);
    glPopMatrix();
    return this;
  }

  */

