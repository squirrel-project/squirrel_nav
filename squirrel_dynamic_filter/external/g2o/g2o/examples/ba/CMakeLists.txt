ADD_EXECUTABLE(ba_demo
  ba_demo.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(ba_demo g2o_core solver_cholmod types_sba)


