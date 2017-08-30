Installation of this package requires the following dependicies

g2o(and all its dependecies): use the version from here: https://github.com/RainerKuemmerle/g2o  and the version that comes with ROS.

libmlpack : https://github.com/mlpack/mlpack

libarmadillo(required by mlpack): http://arma.sourceforge.net/

Change the InputFolder and OutputFolder(params/parameters.xml) accordingly. 

At every compilation, the environment variable `G2O_ROOT` should be set in the
working shell. If not, run
```
	$ export G2O_ROOT=<full-path-to-squirrel_dynamic_filter>/dependencies/g2o
```	
