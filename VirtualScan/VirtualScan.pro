SOURCES += virtualscan.cpp

HEADERS += virtualscan.h

INCLUDEPATH += /opt/ros/indigo/include

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

PROJNAME = VirtualScan
INSTTYPE = SDK
include(RobotSDK_Main.pri)
