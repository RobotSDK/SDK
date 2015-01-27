#-------------------------------------------------
#
# Project created by QtCreator 2014-11-02T13:57:41
#
#-------------------------------------------------

SOURCES += rosinterface.cpp
HEADERS += rosinterface.h

unix {
    INCLUDEPATH += /opt/ros/indigo/include

#    LIBS += -L/opt/ros/indigo/lib -lroscpp
#    LIBS += -L/opt/ros/indigo/lib -lrosconsole
#    LIBS += -L/opt/ros/indigo/lib -lroscpp_serialization
#    LIBS += -L/opt/ros/indigo/lib -lrostime
#    LIBS += -L/opt/ros/indigo/lib -lxmlrpcpp
#    LIBS += -L/opt/ros/indigo/lib -lcpp_common
#    LIBS += -L/opt/ros/indigo/lib -lrosconsole_log4cxx
#    LIBS += -L/opt/ros/indigo/lib -lrosconsole_backend_interface
#    LIBS += -L/opt/ros/indigo/lib -ltf
#    LIBS += -L/opt/ros/indigo/lib -ltf2
#    LIBS += -L/opt/ros/indigo/lib -ltf2_ros
#    LIBS += -L/opt/ros/indigo/lib -lpcl_ros_tf
#    LIBS += -L/opt/ros/indigo/lib -ltf_conversions
#    LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system
}

PROJNAME = ROSInterface
INSTTYPE = SDK

include(RobotSDK_Main.pri)
