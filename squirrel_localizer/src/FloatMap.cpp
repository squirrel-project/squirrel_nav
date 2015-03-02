// FloatMap.cpp --- 
// 
// Filename: FloatMap.cpp
// Description: Map related class
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:16:38 2015 (+0100)
// Version: 
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

#include "squirrel_localizer/FloatMap.h"

#include <sstream>
#include <stdint.h>
#include <vector>
#include <stdio.h>


namespace AISNavigation{

using namespace std;
  
FloatMap::FloatMap(const Vector2i& size_, double resolution_, const Vector2& offset_, float unknown_)
    :_GridMap<float>(size_, resolution_, offset_, unknown_)
{
}

void FloatMap::saveAsPPM(ostream& os, bool equalize) const{
  os << "P6" << endl;
  os << "#resolution " << resolution() << endl;
  os << "#offset "     << offset().x() << " " << offset().y() << endl;
  os << size().x() << " " << size().y() << endl  << 255 << endl;

  int height=this->size().y();
  int width=this->size().x();
  float max=1.;
  if (equalize){
    for (int i=1; i<=height; i++)
      for (int x=0; x<width; x++){
        int y=height-i;
        float k=fabs(cell(Vector2i(x,y)));
        if (k>max)
          max=k;
      }
    if (max!=0)
      max=1./max;
    else
      max=1.;
  }

  for (int i=1; i<=height; i++)
    for (int x=0; x<width; x++){
      int y=height-i;
      float fo=cell(Vector2i(x,y));
      float occ=fo*max;
      unsigned char c=(unsigned char)(255.-255*occ);
      unsigned char r=c, g=c, b=c;
      if (fo==-1.) { // unknown
        b=(unsigned char)(240);
        g=(unsigned char)(220);
        r=(unsigned char)(220);
      } else if (fo==-2.) {// red
        b=(unsigned char)(64);
        g=(unsigned char)(64);
        r=(unsigned char)(255);
        //cerr << "r";
      } else if (fo==-3.){
        b=(unsigned char)(255);
        g=(unsigned char)(64);
        r=(unsigned char)(64);
        //cerr << "b";
      } 
      os.put(r);
      os.put(g);
      os.put(b);
    }
}
  
bool FloatMap::loadFromPPM(istream& is, double res){
  char buf[1024];
  is.getline(buf,1024);
  if (strcmp(buf,"P6"))
    return false;
    
  string tag;
    
  int sx=0;
  int sy=0;
  double ox=0; 
  double oy=0;
  double eqscale=255;
  do {
    is.getline(buf,1024);
    istringstream ls(buf);
    ls >> tag;
    if (tag=="#resolution")
      ls >> res;
    else if (tag=="#offset")
      ls >> ox >> oy;
  } while(buf[0]=='#');
  istringstream ls(buf);
  ls >> sx >> sy;
  cerr << "SIZE:" << sx << " " << sy << endl;
  is.getline(buf,1024);
  eqscale=atof(buf);
  cerr << "maxcol:" << eqscale << endl;
    
  _allocator=_Array2DAllocator<0,0,float>(sx,sy);
  for(int i=sy-1; i>=0; i--)
    for (int j=0; j<sx; j++)
    {
      int r,g,b;
      r=is.get();
      g=is.get();
      b=is.get();
      float f=-1.;
      if (r==g && g==b){
        f=1.-(float(r)+float(g)+float(b))/(3*eqscale);
      } 
      _allocator[j][i]=f;
    }
  _offset=Vector2(ox,oy);
  _resolution=res;
  return true;

}
  
bool FloatMap::fromInt8Vector(const uint32_t& size_x_,const uint32_t& size_y_,
                              const double& offset_x_, const double&  offset_y_,
                              const float& resolution_, const std::vector<signed char>& data ){
  //_allocator=_Array2DAllocator<0,0,float>(size_y_,size_x_);
  _allocator=_Array2DAllocator<0,0,float>(size_x_,size_y_);
  int vecPos = 0;

  for (int i = 0 ; i < data.size(); i++){
    //int x = i/size_y_;
    //int y = i%size_y_;
    int y = i/size_x_;
    int x = i%size_x_;
    char cell = data[i];

    float f=-1.;
    if (cell >= 0){
      f=cell/100.;
    }
    _allocator[x][y]=f;
    //_allocator[y][x]=f;
  }

  _offset=Vector2(offset_x_,offset_y_);
  _resolution=(double_t) resolution_;
  return true;
}

int FloatMap::toChar4Array(unsigned char* buffer, bool equalize) const{
  int height=size().y();
  int width=size().x();

  float max=1.;
  if (equalize){
    for (int x=0; x<width; x++)
      for (int y=0; y<height; y++){
        float k=fabs(cell(Vector2i(x,y)));
        if (k>max)
          max=k;
      }
    if (max!=0)
      max=1./max;
    else
      max=1.;
  }

  int count=0;
  for (int y=0; y<height; y++){
    for (int x=0; x<width; x++){
      float occ=cell(Vector2i(x,y))*max;
      unsigned char c=(unsigned char)(255.-255*occ);
      unsigned char r=c, g=c, b=c;
      if (occ<0){
        b=(unsigned char)(240);
        g=(unsigned char)(220);
        r=(unsigned char)(220);
      }
      *buffer++=r;
      *buffer++=g;
      *buffer++=b;
      *buffer++=255;
      count+=4;
    }
  }
  return count;
}

void FloatMap::computeDistanceMap(FloatMap& dmap, double maxDistance, double minOccupancy, bool markUnknown) const{
  if (dmap.size().x()==size().x() && dmap.size().y()==size().y()){
    dmap.offset()=offset();
    for (int x=0; x<this->size().x(); x++)
      for (int y=0; y<this->size().y(); y++)
        dmap.cell(Vector2i(x,y))=maxDistance;
  } else 
    dmap=FloatMap(size(),resolution(),offset(),maxDistance);
  for (int x=0; x<this->size().x(); x++){
    for (int y=0; y<this->size().y(); y++){
      float f=cell(Vector2i(x,y));
      if (f>minOccupancy){
        dmap.cell(Vector2i(x,y))=0.;
      }
    }
  }

  double ds=sqrt(2)*dmap.resolution();
  double ls=dmap.resolution();
  const int num_passes=3;
  for (int  i=0; i<num_passes; i++){
    for (int x=2; x<dmap.size().x()-1; x++)
      for (int y=2; y<dmap.size().y()-1; y++){
        float mval=maxDistance;
        mval=mval<dmap.cell(Vector2i(x-1,y-1)) + ds ? mval:  dmap.cell(Vector2i(x-1,y-1)) + ds;
        mval=mval<dmap.cell(Vector2i(x-1,y))   + ls ? mval:  dmap.cell(Vector2i(x-1,y))   + ls;
        mval=mval<dmap.cell(Vector2i(x-1,y+1)) + ds ? mval:  dmap.cell(Vector2i(x-1,y+1)) + ds;
        mval=mval<dmap.cell(Vector2i(x,y-1))   + ls ? mval:  dmap.cell(Vector2i(x,y-1))   + ls;
        mval=mval<dmap.cell(Vector2i(x,y+1))   + ls ? mval:  dmap.cell(Vector2i(x,y+1))   + ls;
        mval=mval<dmap.cell(Vector2i(x+1,y-1)) + ds ? mval:  dmap.cell(Vector2i(x+1,y-1)) + ds;
        mval=mval<dmap.cell(Vector2i(x+1,y))   + ls ? mval:  dmap.cell(Vector2i(x+1,y))   + ls;
        mval=mval<dmap.cell(Vector2i(x+1,y+1)) + ds ? mval:  dmap.cell(Vector2i(x+1,y+1)) + ds;
        mval=mval<maxDistance?mval:maxDistance;
        dmap.cell(Vector2i(x,y))=dmap.cell(Vector2i(x,y))<mval ? dmap.cell(Vector2i(x,y)) : mval;
      }
    for (int x=dmap.size().x()-2; x>1; x--)
      for (int y=dmap.size().y()-2; y>1; y--){
        float mval=maxDistance;
        mval=mval<dmap.cell(Vector2i(x-1,y-1)) + ds ? mval:  dmap.cell(Vector2i(x-1,y-1)) + ds;
        mval=mval<dmap.cell(Vector2i(x-1,y))   + ls ? mval:  dmap.cell(Vector2i(x-1,y))   + ls;
        mval=mval<dmap.cell(Vector2i(x-1,y+1)) + ds ? mval:  dmap.cell(Vector2i(x-1,y+1)) + ds;
        mval=mval<dmap.cell(Vector2i(x,y-1))   + ls ? mval:  dmap.cell(Vector2i(x,y-1))   + ls;
        mval=mval<dmap.cell(Vector2i(x,y+1))   + ls ? mval:  dmap.cell(Vector2i(x,y+1))   + ls;
        mval=mval<dmap.cell(Vector2i(x+1,y-1)) + ds ? mval:  dmap.cell(Vector2i(x+1,y-1)) + ds;
        mval=mval<dmap.cell(Vector2i(x+1,y))   + ls ? mval:  dmap.cell(Vector2i(x+1,y))   + ls;
        mval=mval<dmap.cell(Vector2i(x+1,y+1)) + ds ? mval:  dmap.cell(Vector2i(x+1,y+1)) + ds;
        mval=mval<maxDistance?mval:maxDistance;
        dmap.cell(Vector2i(x,y))=dmap.cell(Vector2i(x,y))<mval ? dmap.cell(Vector2i(x,y)) : mval;
      }
  }
  

  if (markUnknown){
    for (int x=0; x<this->size().x(); x++){
      for (int y=0; y<this->size().y(); y++){
        float f=cell(Vector2i(x,y));
        if(f<0.){
          dmap.cell(Vector2i(x,y))=-dmap.cell(Vector2i(x,y));
        }
      }
    }
  }
}
}

// 
// FloatMap.cpp ends here
