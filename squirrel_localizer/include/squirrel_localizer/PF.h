// PF.h --- 
// 
// Filename: PF.h
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

#ifndef SQUIRREL_LOCALIZER_PF_H_
#define SQUIRREL_LOCALIZER_PF_H_

#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>    

#include "squirrel_localizer/OSSpecific.h"
#include "squirrel_localizer/KLDSampling.h"

#include <stdlib.h>

namespace AISNavigation {

/**
   the particle class has to be convertible into numeric data type;
   That means that a particle must define the Numeric conversion operator;
   operator Numeric() const.
   that returns the weight, and the method
   setWeight(Numeric)
   that sets the weight.
*/


/**
   computes the non-log form of the weights from the log-form ones
*/
template <class OutputIterator, class Iterator>
    double toNormalForm(OutputIterator& out, const Iterator & begin, const Iterator & end){
//determine the maximum
double lmax=-std::numeric_limits<double>::max();
for (Iterator it=begin; it!=end; it++){
lmax=lmax>((double)(*it))? lmax: (double)(*it);
}
//convert to raw form
for (Iterator it=begin; it!=end; it++){
*out=exp((double)(*it)-lmax);
out++;
}
return lmax;
}

    /**
       computes the log form of the weights from the normal ones
    */
    template <class OutputIterator, class Iterator,	class Numeric>
    void toLogForm(OutputIterator& out, const Iterator & begin, const Iterator & end, Numeric lmax){
//determine the maximum
for (Iterator it=begin; it!=end; it++){
*out=log((Numeric)(*it))-lmax;
out++;
}
return lmax;
}

        /**
           returns the set of indices resulting from the resampling 
        */
        template <class WeightVector>
        void resample(std::vector<size_t>& indexes, const WeightVector& weights, size_t nparticles=0){
double cweight=0;
	
//compute the cumulative weights
size_t n=0;
for (typename WeightVector::const_iterator it=weights.begin(); it!=weights.end(); ++it){
cweight+=(double)*it;
n++;
}

if (nparticles>0)
  n=nparticles;
size_t n_max = n;
	
//compute the interval
double interval=cweight/n;

//compute the initial target weight
double target=interval*drand48();
//compute the resampled indexes

cweight=0;
indexes.resize(n);
	
n=0;
size_t i=0;
for (typename WeightVector::const_iterator it=weights.begin(); it!=weights.end(); ++it, ++i){
cweight+=(double)* it;
while(cweight>target && n < n_max){
indexes[n++]=i;
target+=interval;
}
}
}


            /**
               returns the set of  resulting from the resampling //TODO change
            */
            template <class Vector>
            void resampleKLD(Vector& dest, const Vector& particles){

std::vector<float>  bins(6,1e10);
bins[0]=bins[1]=0.05;
bins[5]=0.01;

float quantile = 0.8;//0.8
float kld_error = 0.15;//0.1
float min_samples = 200;
float max_samples = 500;

kld_sampling sampler;
sampler.init(quantile,kld_error,bins,min_samples);

int num_samples=0;

std::vector<double> weights(particles.size(), 0);
weights[0] = particles[0];
for(unsigned int i = 1; i < particles.size(); i++){
weights[i] = weights[i-1] + particles[i];
}


while (num_samples < min_samples && num_samples < max_samples) {
unsigned int sample = std::distance(weights.begin(), std::upper_bound(weights.begin(), weights.end(), weights.back()*drand48()));

dest.push_back(particles[sample]);
num_samples++;

//make the sample into a 1D vector because the kld_sampling module
//assumes multivariate distributions.

min_samples=sampler.update(particles[sample]);
}

}



                /**
                   scales a set of weights so that the min log weight is minWeight 
                */
                template <typename WeightVector>
                void normalizeWeights(WeightVector& weights, size_t size, double minWeight){
double wmin=std::numeric_limits<double>::max();
double wmax=-std::numeric_limits<double>::max();
for (size_t i=0; i<size; i++){
wmin=wmin<weights[i]?wmin:weights[i];
wmax=wmax>weights[i]?wmax:weights[i];
}
double min_normalized_value=log(minWeight);
double max_normalized_value=log(1.);
double dn=max_normalized_value-min_normalized_value;
double dw=wmax-wmin;
if (dw==0) dw=1;
double scale=dn/dw;
double offset=-wmax*scale;
for (size_t i=0; i<size; i++){
double w=weights[i];
w=scale*w+offset;
weights[i]=exp(w);
}
}

                    /**
                       repeats in "dest" the entries of "particles" at index indexes[i].
                    */
                    template <typename Vector>
                    void repeatIndexes(Vector& dest, const std::vector<size_t>& indexes, const Vector& particles){
dest.resize(indexes.size());
size_t i=0;
for (std::vector<size_t>::const_iterator it=indexes.begin(); it!=indexes.end(); ++it){
dest[i]=particles[*it];
i++;
}
}

                        /**
                           repeats in "dest" the entries of "particles" at index indexes[i]. Faster version with less memory allocation.
                        */
                        template <typename Vector>
                        void repeatIndexesFast(Vector& dest, const std::vector<size_t>& indexes, const Vector& particles){
size_t i=0;
for (std::vector<size_t>::const_iterator it=indexes.begin(); it!=indexes.end(); ++it){
dest[i]=particles[*it];
i++;
}
                        }

                        /**
                           obsolete
                        */
                        template <typename Vector>
                            void repeatIndexes(Vector& dest, const std::vector<size_t>& indexes2, const Vector& particles, const std::vector<size_t>& indexes){
                          //	assert(indexes.size()==indexes2.size());
                          dest=particles;
                          size_t i=0;
                          for (std::vector<size_t>::const_iterator it=indexes2.begin(); it!=indexes2.end(); ++it){
                            dest[indexes[i]]=particles[*it];
                            i++;
                          }
                        }


                        /**
                           computes the number of effectove particles from the weights
                        */
                        template <class Iterator>
                            double neff(const Iterator& begin, const Iterator& end){
                          double sum=0;
                          for (Iterator it=begin; it!=end; ++it){
                            sum+=*it;
                          }
                          double cum=0;
                          for (Iterator it=begin; it!=end; ++it){
                            double w=*it/sum;
                            cum+=w*w;
                          }
                          return 1./cum;
                        }



                        /**
                           rle compression of the indices of the particles.
                        */
                        template <class OutputIterator, class Iterator>
                            void rle(OutputIterator& out, const Iterator & begin, const Iterator & end){
                          size_t current=0;
                          size_t count=0;
                          for (Iterator it=begin; it!=end; it++){
                            if (it==begin){
                              current=*it;
                              count=1;
                              continue;
                            }
                            if (((size_t)*it) ==current)
                              count++;
                            if (((size_t)*it)!=current){
                              *out=std::make_pair(current,count);
                              out++;
                              current=*it;
                              count=1;
                            }
                          }
                          if (count>0)
                            *out=std::make_pair(current,count);
                          out++;
                        }

}; //namespace AISNavigation

//@}

#endif /* SQUIRREL_LOCALIZER_PF_H_ */

//
// PF.h ends here
        
