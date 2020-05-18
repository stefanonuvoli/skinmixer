############################ CONFIGURATION ############################

#Libraries paths
NUVOLIB_PATH = $$PWD/libs/nuvolib
#EIGEN_PATH = /usr/include/eigen3
#LIBGQLVIEWER_PATH = /usr/lib/x86_64-linux-gnu
VCGLIB_PATH = /usr/include/vcglib
LIBIGL_PATH = /usr/include/libigl

#MLO_PATH = $$PWD/libs/MultiLabelOptimization

#Modules of nuvolib
CONFIG += NVL_MATH NVL_UTILITIES NVL_STRUCTURES NVL_MODELS NVL_IO NVL_VIEWER NVL_VCGLIB NVL_LIBIGL

#App config
TARGET = skinmixer
CONFIG += qt

#Parallel computation
unix:!mac {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -fopenmp
}
macx{
    QMAKE_CXXFLAGS += -Xpreprocessor -fopenmp -lomp -I/usr/local/include
    QMAKE_LFLAGS += -lomp
    LIBS += -L /usr/local/lib /usr/local/lib/libomp.dylib
}


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

#Project files
SOURCES += \
    main.cpp \
    skinmixer/skinmixer.cpp \
    skinmixer/skinmixer_data.cpp \
    skinmixer/skinmixer_utilities.cpp \
    widgets/skinmixer_manager.cpp

HEADERS += \
    skinmixer/skinmixer.h \
    skinmixer/skinmixer_data.h \
    skinmixer/skinmixer_operation.h \
    skinmixer/skinmixer_utilities.h \
    widgets/skinmixer_manager.h

FORMS += \
    widgets/skinmixer_manager.ui

