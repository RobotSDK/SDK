#include "calibrationtoolkit.h"


CalibrationToolkitBase::CalibrationToolkitBase(QWidget * parent)
    : QWidget(parent)
{
    layout=new QHBoxLayout;
    this->setLayout(layout);

    caliblayout=new QVBoxLayout;
    layout->addLayout(caliblayout);

    QLabel * extrinsicmatlabel=new QLabel(EXTRINSICMATSTR);
    caliblayout->addWidget(extrinsicmatlabel);

    extrinsicmat=cv::Mat::eye(4,4,CV_64F);
    extrinsicshow=new QTableWidget;
    caliblayout->addWidget(extrinsicshow);
    setResultShow(extrinsicmat,extrinsicshow);
}

CalibrationToolkitBase::~CalibrationToolkitBase()
{

}

void CalibrationToolkitBase::grabCalibDataSlot()
{
    if(grabCalibData())
    {
        emit calibDataGrabbedSignal();
    }
}

void CalibrationToolkitBase::calibrateSensorSlot()
{
    if(calibrateSensor())
    {
        emit sensorCalibratedSignal();
    }
}

void CalibrationToolkitBase::loadCalibResultSlot()
{
    QString filename=QFileDialog::getOpenFileName(this,"Load Calib",QString(),"YML (*.yml)");
    if(!filename.isEmpty())
    {
        cv::FileStorage fs(filename.toStdString(),cv::FileStorage::READ);
        if(loadCalibResult(fs))
        {
            emit calibResultLoadedSignal();
        }
        fs.release();
    }
}

void CalibrationToolkitBase::saveCalibResultSlot()
{
    QString filename=QFileDialog::getSaveFileName(this,"Save Calib",QString(),"YML (*.yml)");
    if(!filename.isEmpty())
    {
        cv::FileStorage fs(filename.toStdString(),cv::FileStorage::WRITE);
        if(saveCalibResult(fs))
        {
            emit calibResultSavedSignal();
        }
        fs.release();
    }
}

bool CalibrationToolkitBase::loadCalibResult(cv::FileStorage &fs)
{
    fs[EXTRINSICMATSTR]>>extrinsicmat;
    setResultShow(extrinsicmat,extrinsicshow);
    return 1;
}

bool CalibrationToolkitBase::saveCalibResult(cv::FileStorage &fs)
{
    fs<<EXTRINSICMATSTR<<extrinsicmat;
    return 1;
}

void CalibrationToolkitBase::setResultShow(cv::Mat result, QTableWidget *show)
{
    int i,n=result.rows;
    int j,m=result.cols;
    show->clear();
    show->setRowCount(n);
    show->setColumnCount(m);
    for(i=0;i<n;i++)
    {
        for(j=0;j<m;j++)
        {
            show->setItem(i,j,new QTableWidgetItem(QString("%1").arg(result.at<double>(i,j))));
        }
    }
}

cv::Mat CalibrationToolkitBase::getExtrinsicMat()
{
    return extrinsicmat;
}

CalibrateROSCameraChessboard::CalibrateROSCameraChessboard(QString topic, cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrationToolkitBase(parent)
{
    patternsize=patternSize;
    patternnum=patternNum;

    QVBoxLayout * vlayout=new QVBoxLayout;
    layout->addLayout(vlayout);

    timestampshow=new QLabel("Timestamp");
    vlayout->addWidget(timestampshow);

    calibimageshow=new QLabel;
    vlayout->addWidget(calibimageshow);

    calibimagesshow=new QTabWidget;
    vlayout->addWidget(calibimagesshow);

    grid3dpoint.clear();
    grid3dpoints.clear();
    int i,j;
    for(i=0;i<patternnum.height;i++)
    {
        for(j=0;j<patternnum.width;j++)
        {
            cv::Point3f tmpgrid3dpoint;
            tmpgrid3dpoint.x=i*patternsize.height;
            tmpgrid3dpoint.y=j*patternsize.width;
            tmpgrid3dpoint.z=0;
            grid3dpoint.push_back(tmpgrid3dpoint);
        }
    }

    for (i=0; i<256; i++)
    {
        colorTable.push_back(qRgb(i,i,i));
    }

    QLabel * cameramatlabel=new QLabel(CAMERAMAT);
    caliblayout->addWidget(cameramatlabel);

    cameramat=cv::Mat::zeros(3,3,CV_64F);
    cameramatshow=new QTableWidget;
    caliblayout->addWidget(cameramatshow);
    setResultShow(cameramat,cameramatshow);

    QLabel * distcoefflabel=new QLabel(DISTCOEFF);
    caliblayout->addWidget(distcoefflabel);

    distcoeff=cv::Mat::eye(1,8,CV_64F);
    distcoeffshow=new QTableWidget;
    caliblayout->addWidget(distcoeffshow);
    setResultShow(distcoeff,distcoeffshow);

    QLabel * chessboardposelabel=new QLabel(CHESSBOARDPOSE);
    caliblayout->addWidget(chessboardposelabel);

    chessboardposeshow=new QTabWidget;
    caliblayout->addWidget(chessboardposeshow);

    QLabel * reprojectionerrorlabel=new QLabel("Reprojection Error");
    caliblayout->addWidget(reprojectionerrorlabel);

    reprojectionerrorshow=new QLabel;
    caliblayout->addWidget(reprojectionerrorshow);

    camerasub=new ROSSub<sensor_msgs::ImageConstPtr>(topic,1000,10);
    connect(camerasub,SIGNAL(receiveMessageSignal()),this,SLOT(refreshImageSlot()));
    camerasub->startReceiveSlot();
}

CalibrateROSCameraChessboard::~CalibrateROSCameraChessboard()
{

}

void CalibrateROSCameraChessboard::refreshImageSlot()
{
    sensor_msgs::ImageConstPtr msg=camerasub->getMessage();
    if(msg==NULL)
    {
        return;
    }
    QTime timestamp=QTime::fromMSecsSinceStartOfDay((msg->header.stamp.sec%(24*60*60))*1000+msg->header.stamp.nsec/1000000);
    timestampshow->setText(timestamp.toString("HH:mm:ss:zzz"));
    void * data=(void *)(msg->data.data());
    if(QString::fromStdString(msg->encoding)=="rgb8")
    {
        cv::Mat tmpimage=cv::Mat(msg->height,msg->width,CV_8UC3,data);
        cv::cvtColor(tmpimage,calibimage,CV_BGR2RGB);
        QImage img(calibimage.data, calibimage.cols, calibimage.rows, calibimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        calibimageshow->setPixmap(QPixmap::fromImage(img));
    }
    else if(QString::fromStdString(msg->encoding)=="mono8")
    {
        calibimage=cv::Mat(msg->height,msg->width,CV_8UC1,data);
        QImage img(calibimage.data, calibimage.cols, calibimage.rows, calibimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        calibimageshow->setPixmap(QPixmap::fromImage(img));
    }
    else
    {
        calibimageshow->setText("Not Supported");
    }
    return;
}

bool CalibrateROSCameraChessboard::grabCalibData()
{
    camerasub->stopReceiveSlot();
    cv::vector<cv::Point2f> grid2dpoint;
    bool found=cv::findChessboardCorners(calibimage,patternnum,grid2dpoint);
    if(!found)
    {
        camerasub->startReceiveSlot();
        return 0;
    }
    calibimages.push_back(calibimage.clone());
    grid3dpoints.push_back(grid3dpoint);
    grid2dpoints.push_back(grid2dpoint);

    cv::Mat tmpimage=calibimages.back();
    cv::drawChessboardCorners(tmpimage,patternnum,grid2dpoint,1);
    if(tmpimage.type()==CV_8UC3)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        calibimagesshow->addTab(tmpcalibimageshow,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(tmpcalibimageshow);
    }
    else if(tmpimage.type()==CV_8UC1)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        calibimagesshow->addTab(tmpcalibimageshow,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(tmpcalibimageshow);
    }
    camerasub->startReceiveSlot();
    return 1;
}

bool CalibrateROSCameraChessboard::calibrateSensor()
{
    cv::vector<cv::Mat> rvecs;
    cv::vector<cv::Mat> tvecs;
    cv::Size imgsize;
    imgsize.height=calibimage.rows;
    imgsize.width=calibimage.cols;
    cameramat.at<double>(0,2)=imgsize.height/2;
    cameramat.at<double>(1,2)=imgsize.width/2;
    reprojectionerror=cv::calibrateCamera(grid3dpoints,grid2dpoints,imgsize,cameramat,distcoeff,rvecs,tvecs);
    reprojectionerrorshow->setText(QString("%1").arg(reprojectionerror));
    setResultShow(cameramat,cameramatshow);
    setResultShow(distcoeff,distcoeffshow);
    int i,n=rvecs.size();
    chessboardposes.clear();
    chessboardposeshow->clear();
    for(i=0;i<n;i++)
    {
        cv::Mat chessboardpose=cv::Mat::eye(4,4,CV_64F);
        cv::Mat tmprmat=cv::Mat(3,3,CV_64F);
        cv::Rodrigues(rvecs[i],tmprmat);
        int j,k;
        for(j=0;j<3;j++)
        {
            for(k=0;k<3;k++)
            {
                chessboardpose.at<double>(j,k)=tmprmat.at<double>(j,k);
            }
            chessboardpose.at<double>(j,3)=tvecs[i].at<double>(j);
        }
        chessboardposes.push_back(chessboardpose.clone());
        QTableWidget * tmpchessboardposeshow=new QTableWidget;
        chessboardposeshow->addTab(tmpchessboardposeshow,QString("Chessboard_%1").arg(i));
        setResultShow(chessboardpose,tmpchessboardposeshow);
    }
    return 1;
}

bool CalibrateROSCameraChessboard::loadCalibResult(cv::FileStorage &fs)
{
    CalibrationToolkitBase::loadCalibResult(fs);
    fs[CAMERAMAT]>>cameramat;
    fs[DISTCOEFF]>>distcoeff;
    setResultShow(cameramat,cameramatshow);
    setResultShow(distcoeff,distcoeffshow);

    cv::Mat imagenum;
    fs["ImageNum"]>>imagenum;
    int i,n=imagenum.at<u_int16_t>(0);
    chessboardposes.resize(n);
    chessboardposeshow->clear();
    for(i=0;i<n;i++)
    {
        fs[QString("%1_%2").arg(CHESSBOARDPOSE).arg(i).toStdString()]>>chessboardposes[i];
        QTableWidget * tmpchessboardposeshow=new QTableWidget;
        chessboardposeshow->addTab(tmpchessboardposeshow,QString("Chessboard_%1").arg(i));
        setResultShow(chessboardposes[i],tmpchessboardposeshow);
    }
    return 1;
}

bool CalibrateROSCameraChessboard::saveCalibResult(cv::FileStorage &fs)
{
    CalibrationToolkitBase::saveCalibResult(fs);
    fs<<CAMERAMAT<<cameramat;
    fs<<DISTCOEFF<<distcoeff;

    cv::Mat imagenum(1,1,CV_16U);
    imagenum.at<u_int16_t>(0,0)=chessboardposes.size();
    fs<<"ImageNum"<<imagenum;
    int i,n=chessboardposes.size();
    for(i=0;i<n;i++)
    {
        fs<<QString("%1_%2").arg(CHESSBOARDPOSE).arg(i).toStdString()<<chessboardposes[i];
    }
    return 1;
}

cv::Mat CalibrateROSCameraChessboard::getCameraMat()
{
    return cameramat;
}

cv::Mat CalibrateROSCameraChessboard::getDistCoeff()
{
    return distcoeff;
}

cv::Mat CalibrateROSCameraChessboard::getChessboardPose(int id)
{
    return chessboardposes[id];
}

cv::vector<cv::Mat> CalibrateROSCameraChessboard::getChessboardPoses()
{
    return chessboardposes;
}
