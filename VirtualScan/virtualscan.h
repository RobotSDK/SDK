#ifndef VIRTUALSCAN_H
#define VIRTUALSCAN_H

#include<QVector>
#include<sensor_msgs/PointCloud2.h>

class VirtualScan
{
public:
    sensor_msgs::PointCloud2ConstPtr velodynedata;
public:
    int beamnum;
    double step;
    double minfloor;
    double maxceiling;
    double rotation;
    QVector<QVector<double> > dp;
    QVector<int> minfloorid;
    QVector<QVector<int> > pathfloor;
    QVector<QVector<int> > pathceiling;
public:
    VirtualScan();
    virtual ~VirtualScan();
public:
    void calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling, double beamRotation=0);
    void getUpperVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> & virtualScan, QVector<double> & heights);
    void getLowerVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> & virtualScan, QVector<double> & heights);
public:
    int getBeamNum();
    double getHeightStep();
    double getMinFloor();
    double getMaxCeiling();
};

#endif // VIRTUALSCAN_H
