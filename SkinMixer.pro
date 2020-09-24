############################ CONFIGURATION ############################

#Libraries paths
NUVOLIB_PATH = $$PWD/libs/nuvolib
VCGLIB_PATH = $$PWD/libs/vcglib
LIBIGL_PATH = $$PWD/libs/libigl
GUROBI_PATH = /opt/gurobi903/linux64
#EIGEN_PATH = /usr/include/eigen3
#LIBGQLVIEWER_PATH = /usr/lib/x86_64-linux-gnu

DEFINES += SAVE_MESHES

#Modules of nuvolib
CONFIG += NVL_MATH NVL_UTILITIES NVL_STRUCTURES NVL_MODELS NVL_IO NVL_VIEWER NVL_VCGLIB NVL_LIBIGL

#App config
TARGET = skinmixer
CONFIG += qt


######################### FLAGS AND OPTIMIZATION #######################

TEMPLATE = app
CONFIG += c++11

#Debug/release optimization flags
CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next line if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}

#Final release optimization flag
FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS_RELEASE += -O3 -DNDEBUG
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}


############################ PROJECT FILES ############################

#Include nuvolib
include($$NUVOLIB_PATH/nuvolib/nuvolib.pri)
#Include patterns
include($$PWD/libs/patterns/patterns.pri)
#Include patterns
include($$PWD/libs/libiglfields/libiglfields.pri)

#Project files
SOURCES += \
    main.cpp \
    skinmixer/skinmixer.cpp \
    skinmixer/skinmixer_blend_surfaces.cpp \
    skinmixer/skinmixer_data.cpp \
    skinmixer/skinmixer_select.cpp \
    skinmixer/skinmixer_utilities.cpp \
    skinmixer/includes/quad_feasibility.cpp \
    skinmixer/includes/quad_charts.cpp \
    skinmixer/includes/quad_convert.cpp \
    skinmixer/includes/quad_ilp.cpp \
    skinmixer/includes/quad_patch_tracer.cpp \
    skinmixer/includes/quad_patterns.cpp \
    skinmixer/includes/quad_mapping.cpp \
    skinmixer/includes/quad_steps.cpp \
    skinmixer/includes/quad_utils.cpp \
    widgets/skinmixer_manager.cpp \
    skinmixer/skinmixer_blend_skeletons.cpp \
    skinmixer/skinmixer_blend_skinningweights.cpp

HEADERS += \
    skinmixer/skinmixer.h \
    skinmixer/skinmixer_blend_surfaces.h \
    skinmixer/skinmixer_data.h \
    skinmixer/skinmixer_operation.h \
    skinmixer/skinmixer_select.h \
    skinmixer/skinmixer_utilities.h \
    skinmixer/includes/quad_feasibility.h \
    skinmixer/includes/quad_field_tracer.h \
    skinmixer/includes/quad_field_smoother.h \
    skinmixer/includes/quad_convert.h \
    skinmixer/includes/quad_charts.h \
    skinmixer/includes/quad_convert.h \
    skinmixer/includes/quad_ilp.h \
    skinmixer/includes/quad_patch_tracer.h \
    skinmixer/includes/quad_patterns.h \
    skinmixer/includes/quad_mapping.h \
    skinmixer/includes/quad_patch_assembler.h \
    skinmixer/includes/quad_steps.h \
    skinmixer/includes/quad_utils.h \
    widgets/skinmixer_manager.h \
    skinmixer/skinmixer_blend_skeletons.h \
    skinmixer/skinmixer_blend_skinningweights.h

FORMS += \
    widgets/skinmixer_manager.ui


############################ LIBRARIES ############################

#Parallel computation (just in release)
unix:!mac {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -fopenmp
}
macx{
    QMAKE_CXXFLAGS += -Xpreprocessor -fopenmp -lomp -I/usr/local/include
    QMAKE_LFLAGS += -lomp
    LIBS += -L /usr/local/lib /usr/local/lib/libomp.dylib
}

#OpenVDB
CONFIG += c++11
LIBS += -L"usr/local/lib/" -lopenvdb
LIBS += -lblosc -ltbb -lHalf -lboost_thread -lboost_system -lboost_iostreams

#gurobi
INCLUDEPATH += $$GUROBI_PATH/include
LIBS += -L$$GUROBI_PATH/lib -lgurobi_g++5.2 -lgurobi90

#vcg ply
HEADERS += \
    $$VCGLIB_PATH/wrap/ply/plylib.h
SOURCES += \
    $$VCGLIB_PATH/wrap/ply/plylib.cpp
