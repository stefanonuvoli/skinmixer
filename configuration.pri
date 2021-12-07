############################ CONFIGURATION ############################

DEFINES += GLOBAL_TIMES
DEFINES += STEP_TIMES
#DEFINES += SKINMIXER_DEBUG_SAVE_MESHES
#DEFINES += QUADRETOPOLOGY_DEBUG_SAVE_MESHES
#DEFINES += ASSERT_FOR_NUMBER_SIDES
#DEFINES += GUROBI_NON_VERBOSE
#DEFINES += NDEBUG

############################ LIBRARY PATHS ############################

#Internal libraries
NUVOLIB_PATH        = $$PWD/libs/nuvolib/
QUADRETOPOLOGY_PATH = $$PWD/libs/quadretopology/
LIBIGL_PATH         = $$PWD/libs/libigl/
VCGLIB_PATH         = $$PWD/libs/vcglib/

#External libraries
EIGEN_PATH          = /usr/include/eigen3/
BOOST_PATH          = /usr/include/boost/
LIBGQLVIEWER_PATH   = /usr/lib/x86_64-linux-gnu/
OPENVDB_PATH        = /usr/local/lib/
FBXSDK_PATH         = /opt/fbxsdk/
GUROBI_PATH         = /opt/gurobi950/linux64/
GUROBI_COMPILER     = gurobi_g++5.2
GUROBI_LIB          = gurobi95
