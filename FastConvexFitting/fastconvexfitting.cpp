#include "fastconvexfitting.h"

#define MINIMUMDISTANCE 1e-3

FastConvexFitting::FastConvexFitting(QVector<double> & geometryLowerBound, QVector<double> & geometryUpperBound, QVector<double> & geometryStep)
{
    beamsnum=beams.size();
    lb=geometryLowerBound;
    ub=geometryUpperBound;
    step=geometryStep;
    initflag=1;
}

double FastConvexFitting::getGain(double & alpha, double & beta, double & beam)
{
    double gain=1;
    if(alpha>=0&&alpha<=1&&beta>0)
    {
        double distance=(beta-1)*beam;
        double thresh=0.05;
        if(distance>thresh)
        {
            gain=0.8+(exp(-pow(distance-thresh,2)/0.01))*0.4;
        }
        else if(distance>=-thresh&&distance<=thresh)
        {
            gain=1.5;
        }
        else if(distance<-thresh)
        {
            gain=0.1+(exp(-pow(distance+thresh,2)/0.01))*1.4;
        }
    }
    return gain;
}

void FastConvexFitting::updatePosition(double x, double y)
{
    position(0)=x;
    position(1)=y;
}

void FastConvexFitting::updateOrientation(double theta)
{
    orientation(0,0)=cos(theta);orientation(0,1)=-sin(theta);
    orientation(1,0)=sin(theta);orientation(1,1)=cos(theta);
}

void FastConvexFitting::updateScanBeams(QVector<double> & scanBeams)
{
    beams=scanBeams;
    beamsnum=beams.size();

    double PI=3.141592654;
    double density=2*PI/beamsnum;

    points.resize(beamsnum);
    int i;
    for(i=0;i<beamsnum;i++)
    {
        double theta=density*i-PI;
        points[i](0)=beams[i]*cos(theta);
        points[i](1)=beams[i]*sin(theta);
    }
}

bool FastConvexFitting::getEvaluation(Geometry & geometry)
{
    QVector<int> edgeid;
    QVector<int> startid;
    QVector<int> endid;
    if(getBeamRange(geometry,edgeid,startid,endid))
    {
        geometry.score=1;
        int j,m=edgeid.size();
        for(j=0;j<m;j++)
        {
            if(startid[j]>endid[j])
            {
                endid[j]+=beamsnum;
            }
            Eigen::Matrix2d A;
            A.block(0,0,2,1)=orientation*(geometry.edges[edgeid[j]].startcorner-geometry.edges[edgeid[j]].endcorner);
            Eigen::Vector2d B;
            B=position+orientation*(geometry.edges[edgeid[j]].startcorner);
            int k;
            for(k=startid[j];k<=endid[j];k++)
            {
                int tmpid=k;
                if(tmpid>=beamsnum)
                {
                    tmpid-=beamsnum;
                }
                if(beams[tmpid]<=MINIMUMDISTANCE)
                {
                    continue;
                }
                A.block(0,1,2,1)=points[tmpid];
                Eigen::Vector2d x=A.inverse()*B;
                geometry.score*=getGain(x(0),x(1),beams[tmpid]);
            }
        }
        return 1;
    }
    else
    {
        geometry.score=0;
        return 0;
    }
}

bool FastConvexFitting::getFitting(Geometry & geometry)
{
    if(initflag)
    {
        assert(getConfigurations());
        initflag=0;
    }
    if(configurations.size()==0)
    {
        return 0;
    }

    centerposition=-(orientation.inverse()*position);

    int i,n=configurations.size();
    for(i=0;i<n;i++)
    {
        int j,m=configurations[i].edges.size();
        for(j=0;j<m;j++)
        {
            configurations[i].globaledges[j].startcorner=position+orientation*configurations[i].edges[j].startcorner;
            configurations[i].globaledges[j].endcorner=position+orientation*configurations[i].edges[j].endcorner;
        }
    }

    G.score=0;
    for(i=0;i<n;i++)
    {        
        if(getEvaluation(configurations[i])&&G.score<configurations[i].score)
        {
            G=configurations[i];
        }
    }
    if(G.score>0)
    {
        geometry=G;
        return 1;
    }
    else
    {
        return 0;
    }
}

FastRectangleFitting::FastRectangleFitting(QVector<double> & geometryLowerBound, QVector<double> & geometryUpperBound, QVector<double> & geometryStep)
    : FastConvexFitting(geometryLowerBound,geometryUpperBound,geometryStep)
{
    assert(getConfigurations());
    assert(lb.size()==2);
    assert(ub.size()==2);
    assert(step.size()==2);
    int i;
    for(i=0;i<2;i++)
    {
        assert(lb[i]<=ub[i]);
        assert(step[i]>0);
    }
}

bool FastRectangleFitting::getConfigurations()
{
    int i,lsize=(ub[0]-lb[0])/step[0];
    int j,wsize=(ub[1]-lb[1])/step[1];
    configurations.clear();
    for(i=lsize;i>=0;i--)
    {
        for(j=wsize;j>=0;j--)
        {
            Geometry tmpG;
            tmpG.geometry.resize(2);

            tmpG.geometry[0]=lb[0]+i*step[0];
            tmpG.geometry[1]=lb[1]+j*step[1];

            if(tmpG.geometry[0]<tmpG.geometry[1])
            {
                continue;
            }

            tmpG.edges.resize(4);

            tmpG.edges[0].startcorner(0)=tmpG.geometry[0]/2;
            tmpG.edges[0].startcorner(1)=tmpG.geometry[1]/2;
            tmpG.edges[0].endcorner(0)=tmpG.geometry[0]/2;
            tmpG.edges[0].endcorner(1)=-tmpG.geometry[1]/2;

            tmpG.edges[1].startcorner(0)=tmpG.geometry[0]/2;
            tmpG.edges[1].startcorner(1)=-tmpG.geometry[1]/2;
            tmpG.edges[1].endcorner(0)=-tmpG.geometry[0]/2;
            tmpG.edges[1].endcorner(1)=-tmpG.geometry[1]/2;

            tmpG.edges[2].startcorner(0)=-tmpG.geometry[0]/2;
            tmpG.edges[2].startcorner(1)=-tmpG.geometry[1]/2;
            tmpG.edges[2].endcorner(0)=-tmpG.geometry[0]/2;
            tmpG.edges[2].endcorner(1)=tmpG.geometry[1]/2;

            tmpG.edges[3].startcorner(0)=-tmpG.geometry[0]/2;
            tmpG.edges[3].startcorner(1)=tmpG.geometry[1]/2;
            tmpG.edges[3].endcorner(0)=tmpG.geometry[0]/2;
            tmpG.edges[3].endcorner(1)=tmpG.geometry[1]/2;

            tmpG.globaledges.resize(4);

            configurations.push_back(tmpG);
        }
    }
    return 1;
}

bool FastRectangleFitting::getBeamRange(Geometry &configuration, QVector<int> &edgeID, QVector<int> &startID, QVector<int> &endID)
{
    if(centerposition(0)>configuration.geometry[0]/2.0)
    {
        if(centerposition(1)>configuration.geometry[1]/2.0)
        {
            edgeID.resize(2);
            edgeID[0]=0;
            edgeID[1]=3;
        }
        else if(centerposition(1)<-configuration.geometry[1]/2.0)
        {
            edgeID.resize(2);
            edgeID[0]=0;
            edgeID[1]=1;
        }
        else
        {
            edgeID.resize(1);
            edgeID[0]=0;
        }
    }
    else if(centerposition(0)<-configuration.geometry[0]/2.0)
    {
        if(centerposition(1)>configuration.geometry[1]/2.0)
        {
            edgeID.resize(2);
            edgeID[0]=2;
            edgeID[1]=3;
        }
        else if(centerposition(1)<-configuration.geometry[1]/2.0)
        {
            edgeID.resize(2);
            edgeID[0]=1;
            edgeID[1]=2;
        }
        else
        {
            edgeID.resize(1);
            edgeID[0]=2;
        }
    }
    else
    {
        if(centerposition(1)>configuration.geometry[1]/2.0)
        {
            edgeID.resize(1);
            edgeID[0]=3;
        }
        else if(centerposition(1)<-configuration.geometry[1]/2.0)
        {
            edgeID.resize(1);
            edgeID[0]=1;
        }
        else
        {
            return 0;
        }
    }

    double PI=3.141592654;
    double density=2*PI/beamsnum;

    int i,n=edgeID.size();
    startID.resize(n);
    endID.resize(n);
    for(i=0;i<n;i++)
    {
        startID[i]=(atan2(configuration.globaledges[edgeID[i]].startcorner(1),configuration.globaledges[edgeID[i]].startcorner(0))+PI)/density;
        startID[i]=startID[i]>=0?startID[i]:0;
        startID[i]=startID[i]<beamsnum?startID[i]:beamsnum-1;

        endID[i]=(atan2(configuration.globaledges[edgeID[i]].endcorner(1),configuration.globaledges[edgeID[i]].endcorner(0))+PI)/density;
        endID[i]=endID[i]>=0?endID[i]:0;
        endID[i]=endID[i]<beamsnum?endID[i]:beamsnum-1;
    }
    return 1;
}
