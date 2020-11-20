############################ CONFIGURATION ############################

DEFINES += SAVE_MESHES_FOR_DEBUG
#DEFINES += ASSERT_FOR_NUMBER_SIDES
#DEFINES += GUROBI_NON_VERBOSE
DEFINES += NDEBUG

############################ LIBRARY PATHS ############################

#External libraries
OPENVDB_PATH = /usr/local/lib/
GUROBI_PATH = /opt/gurobi903/linux64
GUROBI_COMPILER = gurobi_g++5.2
GUROBI_LIB = gurobi90

#Change if not default
EIGEN_PATH = /usr/include/eigen3
BOOST_PATH = /usr/include/boost
LIBGQLVIEWER_PATH = /usr/lib/x86_64-linux-gnu

#Internal libraries
NUVOLIB_PATH = $$PWD/libs/nuvolib
VCGLIB_PATH = $$PWD/libs/vcglib
LIBIGL_PATH = $$PWD/libs/libigl
QUADRETOPOLOGY_PATH = $$PWD/libs/quadretopology
