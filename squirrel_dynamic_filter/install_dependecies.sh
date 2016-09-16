 #!/bin/bash
 PROJECT_DIR="$(pwd)"
 sudo apt-get install libarmadillo-dev
 mkdir dependicies
 cd "$PROJECT_DIR"/dependicies
 git clone  https://github.com/mlpack/mlpack.git 
 cd mlpack
 mkdir build
 cd build 
 cmake ..
 make-j8 
 sudo make install
 sudo apt-get install libsuitesparse-dev
 sudo apt-get install libqglviewer-dev
 cd "$PROJECT_DIR"/dependicies
 git clone https://github.com/RainerKuemmerle/g2o.git 
 cd g2o
 mkdir build
 cd build
 cmake ..
 make
 echo export G2O_ROOT="$PROJECT_DIR"/dependicies/g2o >> ~/.bashrc
 cp "$PROJECT_DIR"/dependicies/g2o/build/g2o/config.h "$PROJECT_DIR"/dependicies/g2o/g2o/
 
