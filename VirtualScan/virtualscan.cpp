#include "virtualscan.h"

VirtualScan::VirtualScan()
{
    beamnum=1000;
    step=0.3;
    minfloor=-3;
    maxceiling=3;
}

VirtualScan::~VirtualScan()
{
}

void VirtualScan::calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling)
{
    assert(minFloor<maxCeiling);

    beamnum=beamNum;
    step=heightStep;
    minfloor=minFloor;
    maxceiling=maxCeiling;

    double PI=3.141592654;
    double density=2*PI/beamnum;

    int size=int((maxceiling-minfloor)/step+0.5);

    QVector<double> tmpbeam;
    tmpbeam.fill(0,beamnum);
    dp.fill(tmpbeam,size*size);

    char * tmpdata=(char *)(velodynedata->data.data());
    int i,n=velodynedata->height*velodynedata->width;

    //O(N)
    for(i=0;i<n;i++)
    {
        float * point=(float *)(tmpdata+i*velodynedata->point_step);
        int heightid=int((point[2]-minfloor)/step+0.5);
        if(heightid>=0&&heightid<size)
        {
            int id=heightid*size+heightid;
            double theta=atan2(point[1],point[0]);
            int index=int((theta+PI)/density);
            if(index<0)
            {
                index=0;
            }
            else if(index>=beamnum)
            {
                index=beamnum-1;
            }
            double distance=sqrt(point[0]*point[0]+point[1]*point[1]);
            if(distance>0)
            {
                if(dp[id][index]<=0)
                {
                    dp[id][index]=distance;
                }
                else if(dp[id][index]>distance)
                {
                    dp[id][index]=distance;
                }
            }
        }
    }
    //O(S^2)
    for(i=1;i<size;i++)
    {
        int j;
        for(j=i-1;j>=0;j--)
        {
            int lid=j*size+(i-1);
            int did=(j+1)*size+i;
            int id=j*size+i;
            int k;
            for(k=0;k<beamnum;k++)
            {
                if(dp[lid][k]>0&&dp[did][k]>0)
                {
                    dp[id][k]=dp[lid][k]<dp[did][k]?dp[lid][k]:dp[did][k];
                }
                else if(dp[lid][k]>0)
                {
                    dp[id][k]=dp[lid][k];
                }
                else if(dp[did][k]>0)
                {
                    dp[id][k]=dp[did][k];
                }
            }
        }
    }
}

void VirtualScan::getUpperVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> &virtualScan, QVector<double> &heights)
{
    virtualScan.fill(0,beamnum);
    heights.fill(minfloor,beamnum);

    int size=int((maxceiling-minfloor)/step+0.5);

    double delta=fabs(step/tan(theta));

    int i;
    for(i=0;i<beamnum;i++)
    {
        int floorid=0;
        int ceilingid=size-1;
        int id=floorid*size+ceilingid;
        virtualScan[i]=dp[id][i];
        if(virtualScan[i]<=0)
        {
            continue;
        }
        bool jumpflag=1;
        while(floorid<ceilingid)
        {
            id=(floorid+1)*size+ceilingid;
            if(dp[id][i]>0)
            {
                if(jumpflag&&fabs(dp[id][i]-virtualScan[i])<delta)
                {
                    virtualScan[i]=dp[id][i];
                    floorid++;
                    if(minfloor+floorid*step>maxFloor)
                    {
                        break;
                    }
                    else
                    {
                        continue;
                    }
                }
                jumpflag=0;
                if(dp[id][i]-virtualScan[i]>=delta)
                {
                    virtualScan[i]=dp[id][i];
                    floorid++;
                }
                else
                {
                    double tmpdis=dp[id][i];
                    int j;
                    for(j=ceilingid-1;j>=floorid;j--)
                    {
                        id=floorid*size+j;
                        if(dp[id][i]>tmpdis)
                        {
                            virtualScan[i]=dp[id][i];
                            ceilingid=j;
                            break;
                        }
                        else if(minfloor+(j+1)*step<minCeiling)
                        {
                            j=floorid;
                        }
                    }
                    if(j<floorid)
                    {
                        virtualScan[i]=tmpdis;
                        floorid++;
                        break;
                    }
                }
            }
            else
            {
                virtualScan[i]=dp[id][i];
                floorid++;
                break;
            }
        }
        heights[i]=minfloor+(floorid-1+0.5)*step;
    }
}

void VirtualScan::getLowerVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> &virtualScan, QVector<double> &heights)
{
    virtualScan.fill(0,beamnum);
    heights.fill(minfloor,beamnum);

    int size=int((maxceiling-minfloor)/step+0.5);

    double delta=fabs(step/tan(theta));

    int i;
    for(i=0;i<beamnum;i++)
    {
        int floorid=0;
        int ceilingid=size-1;
        int id=floorid*size+ceilingid;
        virtualScan[i]=dp[id][i];
        if(virtualScan[i]<=0)
        {
            continue;
        }
        bool jumpflag=1;
        while(floorid<ceilingid)
        {
            id=(floorid)*size+(ceilingid-1);
            if(dp[id][i]>0)
            {
                if(jumpflag&&fabs(dp[id][i]-virtualScan[i])<delta)
                {
                    virtualScan[i]=dp[id][i];
                    ceilingid--;
                    if(minfloor+(ceilingid+1)*step<minCeiling)
                    {
                        break;
                    }
                    else
                    {
                        continue;
                    }
                }
                jumpflag=0;
                if(dp[id][i]-virtualScan[i]>=delta)
                {
                    virtualScan[i]=dp[id][i];
                    ceilingid--;
                }
                else
                {
                    double tmpdis=dp[id][i];
                    int j;
                    for(j=floorid+1;j<=ceilingid;j++)
                    {
                        id=j*size+ceilingid;
                        if(dp[id][i]>tmpdis)
                        {
                            virtualScan[i]=dp[id][i];
                            floorid=j;
                            break;
                        }
                        else if(minfloor+j*step>maxFloor)
                        {
                            j=ceilingid;
                        }
                    }
                    if(j>ceilingid)
                    {
                        virtualScan[i]=tmpdis;
                        ceilingid--;
                        break;
                    }
                }
            }
            else
            {
                virtualScan[i]=dp[id][i];
                ceilingid--;
                break;
            }
        }
        heights[i]=minfloor+(ceilingid+0.5)*step;
    }
}

int VirtualScan::getBeamNum()
{
    return beamnum;
}

double VirtualScan::getHeightStep()
{
    return step;
}

double VirtualScan::getMinFloor()
{
    return minfloor;
}

double VirtualScan::getMaxCeiling()
{
    return maxceiling;
}
