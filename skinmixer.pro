############################ TARGET AND FLAGS ############################

#App config
TARGET = skinmixer
TEMPLATE = app
CONFIG += c++17
CONFIG += qt
CONFIG -= app_bundle
QT += core gui opengl widgets

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
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}

macx {
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.13
    QMAKE_MAC_SDK = macosx10.13
}


############################ LIBRARIES ############################

#Setting library paths and configuration
include(configuration.pri)

#nuvolib (it includes vcglib, libigl, eigen, libqglviewer ...)
include($$NUVOLIB_PATH/nuvolib.pri)

#Quad retopology
include($$QUADRETOPOLOGY_PATH/quadretopology.pri)

#Boost
INCLUDEPATH += $$BOOST_PATH

#OpenVDB
CONFIG += c++11
LIBS += -L$$OPENVDB_PATH -lopenvdb

#gurobi
INCLUDEPATH += $$GUROBI_PATH/include
LIBS += -L$$GUROBI_PATH/lib -l$$GUROBI_COMPILER -l$$GUROBI_LIB
DEFINES += GUROBI_DEFINED

LIBS += -lblosc -ltbb -lHalf -lboost_thread -lboost_system -lboost_iostreams

#Parallel computation
unix:!mac {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -fopenmp
}
#macx{
#    QMAKE_CXXFLAGS += -Xpreprocessor -fopenmp -lomp -I/usr/local/include
#    QMAKE_LFLAGS += -lomp
#    LIBS += -L /usr/local/lib /usr/local/lib/libomp.dylib
#}

win32{
    DEFINES += NOMINMAX # Awful problem with windows..
    DEFINES *= _USE_MATH_DEFINES
    DEFINES *= _SCL_SECURE_NO_DEPRECATE
    QMAKE_CXXFLAGS *= /bigobj
}

############################ PROJECT FILES ############################

#Project files
SOURCES += \
    main.cpp \
    skinmixer/internal/skinmixer_field.cpp \
    skinmixer/skinmixer.cpp \
    skinmixer/internal/skinmixer_attach_borders.cpp \
    skinmixer/internal/skinmixer_morphological_operations.cpp \
    skinmixer/skinmixer_blend_animations.cpp \
    skinmixer/skinmixer_blend_surfaces.cpp \
    skinmixer/skinmixer_data.cpp \
    skinmixer/skinmixer_select.cpp \
    skinmixer/skinmixer_utilities.cpp \
    widgets/skinmixer_manager.cpp \
    skinmixer/skinmixer_blend_skeletons.cpp \
    skinmixer/skinmixer_blend_skinningweights.cpp

HEADERS += \
    skinmixer/internal/skinmixer_field.h \
    skinmixer/skinmixer.h \
    skinmixer/internal/skinmixer_attach_borders.h \
    skinmixer/internal/skinmixer_morphological_operations.h \
    skinmixer/skinmixer_blend_animations.h \
    skinmixer/skinmixer_blend_surfaces.h \
    skinmixer/skinmixer_data.h \
    skinmixer/skinmixer_select.h \
    skinmixer/skinmixer_utilities.h \
    widgets/skinmixer_manager.h \
    skinmixer/skinmixer_blend_skeletons.h \
    skinmixer/skinmixer_blend_skinningweights.h

FORMS += \
    widgets/skinmixer_manager.ui

