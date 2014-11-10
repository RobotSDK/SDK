#include "calibrationtoolkit.h"


CalibrationToolkitBase::CalibrationToolkitBase(QWidget * parent)
    : QWidget(parent)
{
    layout=new QHBoxLayout;
    this->setLayout(layout);

    caliblayout=new QVBoxLayout;
    layout->addLayout(caliblayout);

    QHBoxLayout * hlayout=new QHBoxLayout;
    caliblayout->addLayout(hlayout);

    grabbutton=new QPushButton("Grab");
    hlayout->addWidget(grabbutton);
    calibbutton=new QPushButton("Calibrate");
    hlayout->addWidget(calibbutton);

    hlayout->addStretch();

    loadbutton=new QPushButton("Load");
    hlayout->addWidget(loadbutton);
    savebutton=new QPushButton("Save");
    hlayout->addWidget(savebutton);

    QLabel * extrinsicmatlabel=new QLabel(EXTRINSICMATSTR);
    caliblayout->addWidget(extrinsicmatlabel);

    extrinsicmat=cv::Mat::eye(4,4,CV_32F);
    extrinsicshow=new QTableWidget;
    caliblayout->addWidget(extrinsicshow);
    setResultShow(extrinsicmat,extrinsicshow);

    connect(grabbutton,SIGNAL(clicked()),this,SLOT(grabCalibDataSlot()));
    connect(calibbutton,SIGNAL(clicked()),this,SLOT(calibrateSensorSlot()));
    connect(loadbutton,SIGNAL(clicked()),this,SLOT(loadCalibResultSlot()));
    connect(savebutton,SIGNAL(clicked()),this,SLOT(saveCalibResultSlot()));
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
    show->setRowCount(n);
    show->setColumnCount(m);
    for(i=0;i<n;i++)
    {
        for(j=0;j<m;j++)
        {
            show->item(i,j)->setText(QString("%1").arg(result.at<float>(i,j)));
        }
    }
}

cv::Mat CalibrationToolkitBase::getExtrinsicMat()
{
    return extrinsicmat;
}

CalibrateCameraChessboard::CalibrateCameraChessboard(QString topic, cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrationToolkitBase(parent)
{
    QHBoxLayout * hlayout=new QHBoxLayout;
    layout->addLayout(hlayout);

    timestampshow=new QLabel("Timestamp");
    hlayout->addWidget(timestampshow);

    calibimageshow=new QLabel("Image");
    hlayout->addWidget(calibimageshow);

    patternsize=patternSize;
    patternnum=patternNum;
    grid3dpoints.clear();
    int i,j;
    for(i=0;i<patternnum.height;i++)
    {
        for(j=0;j<patternnum.width;j++)
        {
            cv::Point3f grid3dpoint;
            grid3dpoint.x=i*patternsize.height;
            grid3dpoint.y=j*patternsize.width;
            grid3dpoint.z=0;
            grid3dpoints.push_back(grid3dpoint);
        }
    }

    for (i=0; i<256; i++)
    {
        colorTable.push_back(qRgb(i,i,i));
    }

    QLabel * cameramatlabel=new QLabel(CAMERAMAT);
    caliblayout->addWidget(cameramatlabel);

    cameramat=cv::Mat::eye(3,3,CV_32F);
    cameramatshow=new QTableWidget;
    caliblayout->addWidget(cameramatshow);
    setResultShow(cameramat,cameramatshow);

    QLabel * distcoefflabel=new QLabel(DISTCOEFF);
    caliblayout->addWidget(distcoefflabel);

    distcoeff=cv::Mat::eye(1,5,CV_32F);
    distcoeffshow=new QTableWidget;
    caliblayout->addWidget(distcoeffshow);
    setResultShow(distcoeff,distcoeffshow);

    QLabel * chessboardposelabel=new QLabel(CHESSBOARDPOSE);
    caliblayout->addWidget(chessboardposelabel);

    chessboardpose=cv::Mat::eye(4,4,CV_32F);
    chessboardposeshow=new QTableWidget;
    caliblayout->addWidget(chessboardposeshow);
    setResultShow(chessboardpose,chessboardposeshow);

    grabbutton->setEnabled(1);
    calibbutton->setEnabled(0);
    loadbutton->setEnabled(1);
    savebutton->setEnabled(1);

    camerasub=new ROSSub<sensor_msgs::ImageConstPtr>(topic,1000,10);
    connect(camerasub,SIGNAL(receiveMessageSignal()),this,SLOT(refreshImageSlot()));
    camerasub->startReceiveSlot();
}

CalibrateCameraChessboard::~CalibrateCameraChessboard()
{

}

void CalibrateCameraChessboard::refreshImageSlot()
{
    sensor_msgs::ImageConstPtr msg=camerasub->getMessage();
    if(msg==NULL)
    {
        return;
    }
    QTime timestamp=QTime::fromMSecsSinceStartOfDay(msg->header.stamp.sec*1000+msg->header.stamp.nsec/1000);
    timestampshow->setText(timestamp.toString("HH:mm:ss:zzz"));
    void * data=(void *)(msg->data.data());
    if(QString::fromStdString(msg->encoding)=="rgb8")
    {
        calibimage=cv::Mat(msg->height,msg->width,CV_8UC3,data);
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

bool CalibrateCameraChessboard::grabCalibData()
{
    if(grabbutton->text()=="Grab")
    {
        grabbutton->setText("Camera");
        calibbutton->setEnabled(1);
        camerasub->stopReceiveSlot();
    }
    else
    {
        grabbutton->setText("Grab");
        calibbutton->setEnabled(0);
        camerasub->startReceiveSlot();
    }
}

bool CalibrateCameraChessboard::calibrateSensor()
{
    bool found=cv::findChessboardCorners(calibimage,patternsize,grid2dpoints);
    if(!found)
    {
        return 0;
    }
    cv::Mat rvecs;
    cv::Mat tvecs;
    cv::Size imgsize;
    imgsize.height=calibimage.rows;
    imgsize.width=calibimage.cols;
    reprojectionerror=cv::calibrateCamera(grid3dpoints,grid2dpoints,imgsize,cameramat,distcoeff,rvecs,tvecs);
    cv::Rodrigues(rvecs,chessboardpose(cv::Rect(0,0,3,3)));
    tvecs.copyTo(chessboardpose(cv::Rect(0,3,3,1)));
    setResultShow(cameramat,cameramatshow);
    setResultShow(distcoeff,distcoeffshow);
    setResultShow(chessboardpose,chessboardposeshow);
    cv::Mat tmpimage=calibimage.clone();
    cv::drawChessboardCorners(tmpimage,patternsize,grid2dpoints,1);
    if(tmpimage.type()==CV_8UC3)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        calibimageshow->setPixmap(QPixmap::fromImage(img));
    }
    else if(tmpimage.type()==CV_8UC1)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        calibimageshow->setPixmap(QPixmap::fromImage(img));
    }
    else
    {
        calibimageshow->setText("Not Supported");
    }
    return 1;
}

bool CalibrateCameraChessboard::loadCalibResult(cv::FileStorage &fs)
{
    CalibrationToolkitBase::loadCalibResult(fs);
    fs[CAMERAMAT]>>cameramat;
    fs[DISTCOEFF]>>distcoeff;
    fs[CHESSBOARDPOSE]>>chessboardpose;
    setResultShow(cameramat,cameramatshow);
    setResultShow(distcoeff,distcoeffshow);
    setResultShow(chessboardpose,chessboardposeshow);
    return 1;
}

bool CalibrateCameraChessboard::saveCalibResult(cv::FileStorage &fs)
{
    CalibrationToolkitBase::saveCalibResult(fs);
    fs<<CAMERAMAT<<cameramat;
    fs<<DISTCOEFF<<distcoeff;
    fs<<CHESSBOARDPOSE<<chessboardpose;
    return 1;
}

cv::Mat CalibrateCameraChessboard::getCameraMat()
{
    return cameramat;
}

cv::Mat CalibrateCameraChessboard::getDistCoeff()
{
    return distcoeff;
}

cv::Mat CalibrateCameraChessboard::getChessboardPose()
{
    return chessboardpose;
}
