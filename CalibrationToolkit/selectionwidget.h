#ifndef SELECTIONWIDGET_H
#define SELECTIONWIDGET_H

#include<qlabel.h>
#include<QVector>
#include<qrgb.h>
#include<QMouseEvent>

#include<opencv2/opencv.hpp>

#include<Eigen/Dense>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/normal_3d_omp.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>

#include<glviewer.h>

class PlaneExtractor : public GLViewer
{
    Q_OBJECT
public:
    PlaneExtractor(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, int neighborNum=50, double neighborRadius=0.2, double distanceThreshold=0.05, QWidget * parent=0);
protected:
    int pointsid;
    int neighbornum;
    double neighborradius;
    double distance;
    sensor_msgs::PointCloud2ConstPtr pointsptr;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    pcl::PointCloud<pcl::Normal> normals;
    GLuint pointsdisplaylist;
    GLuint selecteddisplaylist;
    GLuint mousedisplaylist;
    bool extracted;
protected slots:
    void mousePositionSlot(QMouseEvent * event, CAMERAPARAMETERS * parameters);
signals:
    void extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr extraction, int id);
protected:
    void drawMouse(Eigen::Vector3d point, Eigen::Vector3d norm, Eigen::Vector3d direction,Eigen::Vector3d nearestpoint);
    void extractPlane(Eigen::Vector3d seed, Eigen::Matrix3d eigenvectors);
};


#endif // SELECTIONWIDGET_H
