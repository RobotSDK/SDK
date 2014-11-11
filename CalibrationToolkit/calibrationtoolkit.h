#ifndef CALIBRATIONTOOLKIT_H
#define CALIBRATIONTOOLKIT_H

#include<qwidget.h>
#include<QHBoxLayout>
#include<QVBoxLayout>
#include<qlabel.h>
#include<qpushbutton.h>
#include<qtablewidget.h>
#include<qfiledialog.h>
#include<qimage.h>
#include<qdatetime.h>
#include<qtabwidget.h>

#include<opencv2/opencv.hpp>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<pcl/search/kdtree.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<rosinterface.h>

#define EXTRINSICMATSTR "ExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define CHESSBOARDPOSE "ChessboardPose"
#define CHESSBOARDVIEWNUM "ChessboardViewNUM"
#define REPROJECTIONERROR "ReprojectionError"

class CalibrationToolkitBase : public QWidget
{
    Q_OBJECT
public:
    CalibrationToolkitBase(QWidget * parent=0);
protected:
    QHBoxLayout * layout;
    QVBoxLayout * caliblayout;
    cv::Mat extrinsicmat;
    QTableWidget * extrinsicshow;
public slots:
    void grabCalibDataSlot();
    void calibrateSensorSlot();
    void loadCalibResultSlot();
    void saveCalibResultSlot();
signals:
    void calibDataGrabbedSignal();
    void calibDataGrabbedErrorSignal();
    void sensorCalibratedSignal();
    void sensorCalibratedErrorSignal();
    void calibResultLoadedSignal();
    void calibResultLoadedErrorSignal();
    void calibResultSavedSignal();
    void calibResultSavedErrorSignal();
protected:
    virtual bool grabCalibData()=0;
    virtual bool calibrateSensor()=0;
    virtual bool loadCalibResult(cv::FileStorage & fs);
    virtual bool saveCalibResult(cv::FileStorage & fs);
protected:
    void setResultShow(cv::Mat result, QTableWidget * show);
public:
    cv::Mat getExtrinsicMat();
};

class CalibrateCameraBase : public CalibrationToolkitBase
{
    Q_OBJECT
public:
    CalibrateCameraBase(QWidget * parent=0);
protected:
    QVBoxLayout * imagelayout;
    QLabel * timestampshow;
    cv::Mat calibimage;
    QLabel * calibimageshow;
    QVector<QRgb> colorTable;

    cv::Mat cameramat;
    QTableWidget * cameramatshow;
    cv::Mat distcoeff;
    QTableWidget * distcoeffshow;
protected slots:
    void refreshImageSlot();
signals:
    void imageRefreshedSignal();
    void imageRefreshedErrorSignal();
protected:
    virtual bool refreshImage()=0;
    bool loadCalibResult(cv::FileStorage & fs);
    bool saveCalibResult(cv::FileStorage & fs);
public:
    cv::Mat getCameraMat();
    cv::Mat getDistCoeff();
};

class CalibrateCameraChessboardBase : public CalibrateCameraBase
{
    Q_OBJECT
public:
    CalibrateCameraChessboardBase(cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
protected:
    cv::Size2i patternnum;
    cv::Size2f patternsize;

    cv::vector<cv::Point3f> grid3dpoint;
    cv::vector<cv::vector<cv::Point3f> > grid3dpoints;
    cv::vector<cv::vector<cv::Point2f> > grid2dpoints;

    cv::vector<cv::Mat> calibimages;
    QTabWidget * calibimagesshow;

    cv::vector<cv::Mat> chessboardposes;
    QTabWidget * chessboardposeshow;

    double reprojectionerror;
    QLabel * reprojectionerrorshow;
protected:
    bool loadCalibResult(cv::FileStorage & fs);
    bool saveCalibResult(cv::FileStorage & fs);
public:
    int getChessboardNum();
    cv::vector<cv::Mat> getChessboardPoses();
    cv::Mat getChessboardPose(int id);
    cv::Mat getCalibImage(int id);
};

class CalibrateCameraChessboardROS : public CalibrateCameraChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraChessboardROS(QString topic, u_int32_t queueSize, int interval, cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
    ~CalibrateCameraChessboardROS();
protected:
    ROSSub<sensor_msgs::ImageConstPtr> * camerasub;
protected:
    bool refreshImage();
    bool grabCalibData();
    bool calibrateSensor();
};

class CalibrateCameraVelodyneChessboardBase : public CalibrateCameraChessboardBase
{
    Q_OBJECT
public:
    CalibrateCameraVelodyneChessboardBase(cv::Size2f patternSize, cv::Size2i patternNum, QWidget * parent=0);
protected:

};

#endif // CALIBRATIONTOOLKIT_H
