#!/bin/bash

PROJECT_DIR="$(pwd)"
NUM_MAKE_THREADS=$(($(nproc)+1))

## Install Armadillo.
sudo apt-get install libarmadillo-dev -y

## Install MLPACK.
mkdir dependencies
cd "$PROJECT_DIR"/dependencies
git clone  https://github.com/mlpack/mlpack.git 
cd mlpack
mkdir build
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j${NUM_MAKE_THREADS}
sudo make install

## Install suitesparse.
sudo apt-get install libsuitesparse-dev -y

## Intall gl viewer.
sudo apt-get install libqglviewer-dev -y

## Install g2o.
cd "$PROJECT_DIR"/dependencies
git clone https://github.com/RainerKuemmerle/g2o.git 
cd g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j${NUM_MAKE_THREADS}
export G2O_ROOT="$PROJECT_DIR"/dependencies/g2o
echo export G2O_ROOT="$PROJECT_DIR"/dependencies/g2o >> ~/.bashrc
. ~/.bashrc
cp "$PROJECT_DIR"/dependencies/g2o/build/g2o/config.h "$PROJECT_DIR"/dependencies/g2o/g2o/
 
