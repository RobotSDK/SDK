SOURCES += modelbasedtracking.cpp

HEADERS += modelbasedtracking.h

INCLUDEPATH += /usr/local/include/mrpt/base/include

LIBS += -L/usr/local/lib -lmrpt-base

PROJNAME = ModelBasedTracking
INSTTYPE = SDK
include(RobotSDK_Main.pri)
