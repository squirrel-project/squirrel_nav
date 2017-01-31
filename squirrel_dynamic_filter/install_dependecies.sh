 #!/bin/bash
 NUM_MAKE_THREADS=$(nproc)
 PROJECT_DIR="$(pwd)"
 mkdir dependencies
 cd "$PROJECT_DIR"/dependencies
 git clone https://github.com/RainerKuemmerle/g2o.git 
 cd g2o
 mkdir build
 cd build
 cmake ..
 make -j${NUM_MAKE_THREADS}
 sudo make install
 echo export G2O_ROOT="$PROJECT_DIR"/dependencies/g2o >> ~/.bashrc
 . ~/.bashrc
 cp "$PROJECT_DIR"/dependencies/g2o/build/g2o/config.h "$PROJECT_DIR"/dependencies/g2o/g2o/
