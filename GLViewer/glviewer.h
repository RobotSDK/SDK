#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <QtOpenGL/QtOpenGL>
#include <QWidget>
#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QGLWidget>

struct CAMERAPARAMETERS
{
    double viewAngle;
    double viewheight;
    double minView, maxView;
    int width,height;
    Eigen::Vector4d background;
	GLfloat lightambient[4];
    Eigen::Matrix4d transform;
	GLfloat eye[4];
    double tspeed,rspeed;
    int pointsize;
};

struct DISPLAYLIST
{
    bool show;
    GLuint listid;
	Eigen::Matrix4d transform;
	Eigen::Matrix4d scale;
};

class GLViewer : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLViewer(QWidget *parent = 0);
    ~GLViewer();
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void keyPressEvent(QKeyEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void wheelEvent(QWheelEvent * event);
    void mouseMoveEvent(QMouseEvent * event);
signals:
    void mousePosition(int mx, int my, Eigen::Vector3d eye, double minview, double maxview, QMouseEvent *event);
private:
    CAMERAPARAMETERS parameters;
    std::vector<DISPLAYLIST> displaylist;
    bool bperspective;
private:
    void setProjection();
public:
    void addDisplayList(GLuint listid);
    void addDisplayLists(std::vector<GLuint> & listids);
    void enableShow(GLuint listid, bool show, bool islistid=0);
    void deleteDisplayList(GLuint listid, bool islistid=0);
    void clearDisplayList();
    int listSize();
    void setCameraPose(Eigen::Matrix4d transform);
    Eigen::Matrix4d getCameraPose();
    void setBackground(QColor color);
	void setDisplayListScale(GLuint listid, double sx, double sy, double sz, bool islistid=1);
	void setDisplayListRotation(GLuint listid, double rx, double ry, double rz, bool islistid=1);
	void setDisplayListTranslation(GLuint listid, double tx, double ty, double tz, bool islistid=1);
	void setDisplayListTransform(GLuint listid, Eigen::Matrix4d transform, bool islistid=1);
};

#endif
