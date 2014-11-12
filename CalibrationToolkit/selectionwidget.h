#ifndef SELECTIONWIDGET_H
#define SELECTIONWIDGET_H

#include<qlabel.h>
#include<QVector>
#include<qrgb.h>
#include<QMouseEvent>

#include<opencv2/opencv.hpp>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>

#include<glviewer.h>

class PlaneExtraction : public InteractiveGLViewer
{
    Q_OBJECT
public:
    PlaneExtraction(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, QWidget * parent=0);
protected:
    int pointsid;
    sensor_msgs::PointCloud2ConstPtr pointsptr;
//    pcl::PointCloud<pcl::PointXYZI> points;
//    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    GLuint pointsdisplaylist;
    GLuint selecteddisplaylist;
    GLuint surfacedisplaylist;
public slots:
    void handleInteractionSlot();
signals:
    void selectionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr selection, int id);
};


#endif // SELECTIONWIDGET_H
