#ifndef FASTVIRTUALSCAN_H
#define FASTVIRTUALSCAN_H

#include<QVector>
#include<QtAlgorithms>
#include<sensor_msgs/PointCloud2.h>

struct SimpleVirtualScan
{
    int rotid;
    double rotlength;
    double rotheight;
    double length;
    double height;
};

class FastVirtualScan
{
public:
    sensor_msgs::PointCloud2ConstPtr velodyne;
public:
    int beamnum;
    double step;
    double minfloor;
    double maxceiling;
    double rotation;
    QVector<QVector<SimpleVirtualScan> > svs;
    QVector<double> minheights;
    QVector<double> maxheights;
public:
    FastVirtualScan();
    virtual ~FastVirtualScan();
public:
    void calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling, double beamRotation=0);
    void getVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> & virtualScan);
};

#endif // FASTVIRTUALSCAN_H
