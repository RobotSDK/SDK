SOURCES += modelbasedtracking.cpp

HEADERS += modelbasedtracking.h

INCLUDEPATH += /usr/local/include/mrpt/base/include
INCLUDEPATH +=/usr/local/include/mrpt/mrpt-config
LIBS += -L/usr/local/lib -lmrpt-base

INCLUDEPATH += /usr/include
LIBS += -L/usr/lib/x86_64-linux-gnu -lnlopt

INCLUDEPATH +=/usr/include/eigen3

PROJNAME = ModelBasedTracking
INSTTYPE = SDK
include(RobotSDK_Main.pri)
