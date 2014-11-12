#include"selectionwidget.h"

PlaneExtraction::PlaneExtraction(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, QWidget *parent)
    : InteractiveGLViewer(100,parent)
{
    pointsid=id;
    pointsptr=velodynePoints;

    this->makeCurrent();

    pointsdisplaylist=glGenLists(1);
    selecteddisplaylist=glGenLists(1);
    surfacedisplaylist=glGenLists(1);

    this->addDisplayList(pointsdisplaylist);
    this->addDisplayList(selecteddisplaylist);
    this->addDisplayList(surfacedisplaylist);

    int i,n=pointsptr->height*pointsptr->width;
    QVector<float> colors(3*n);
    char * data=(char *)(pointsptr->data.data());
    for(i=0;i<n;i++)
    {
        float * base=(float *)(data+i*pointsptr->point_step);
        colors[i*3+0]=base[4]/255.0;
        colors[i*3+1]=base[4]/255.0;
        colors[i*3+2]=base[4]/255.0;
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3,GL_FLOAT,pointsptr->point_step,pointsptr->data.data());
    glColorPointer(3,GL_FLOAT,3*sizeof(float),colors.data());


    glNewList(pointsdisplaylist,GL_COMPILE);

    glDrawArrays(GL_POINTS,0,n);

    glEndList();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    this->update();

    connect(mousecameraeventqueue,SIGNAL(interactiveSignal()),this,SLOT(handleInteractionSlot()));
}

void PlaneExtraction::handleInteractionSlot()
{

}
