SOURCES += fastvirtualscan.cpp

HEADERS += fastvirtualscan.h

INCLUDEPATH += /opt/ros/indigo/include

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

PROJNAME = FastVirtualScan
INSTTYPE = SDK
include(RobotSDK_Main.pri)
