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

void VirtualScan::calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling, double beamRotation)
{
    assert(minFloor<maxCeiling);

    beamnum=beamNum;
    step=heightStep;
    minfloor=minFloor;
    maxceiling=maxCeiling;
    rotation=beamRotation;

    double PI=3.141592654;
    double density=2*PI/beamnum;

    int size=int((maxceiling-minfloor)/step+0.5);


    pathfloor.clear();
    pathfloor.resize(beamnum);
    pathceiling.clear();
    pathceiling.resize(beamnum);
    minfloorid.resize(beamnum);

    dp.clear();
    dp.resize(beamnum);
    {
        int i;
        for(i=0;i<beamnum;i++)
        {
            dp[i].resize(size*size);
            dp[i].fill(0);
        }
    }

    {
        char * tmpdata=(char *)(velodynedata->data.data());
        int i,n=velodynedata->height*velodynedata->width;

        double c=cos(rotation);
        double s=sin(rotation);

        //O(P)
        for(i=0;i<n;i++)
        {
            float * pointdata=(float *)(tmpdata+i*velodynedata->point_step);
            float point[3];
            point[0]=pointdata[0];
            point[1]=pointdata[1];
            double distance=sqrt(point[0]*point[0]+point[1]*point[1]);
            point[2]=distance*s+pointdata[2]*c;
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

                if(distance>0)
                {
                    if(dp[index][id]<=0)
                    {
                        dp[index][id]=distance;
                    }
                    else if(dp[index][id]>distance)
                    {
                        dp[index][id]=distance;
                    }
                }
            }
        }
    }
    {
#pragma omp parallel for \
    default(shared) \
    schedule(dynamic,10)
        for(int i=0;i<beamnum;i++)
        {
            int j;
            bool flag=1;
            int startid;
            //O(M)
            double minlength=0;
            minfloorid[i]=0;
            for(j=0;j<size;j++)
            {
                int id=j*size+j;
                if(flag)
                {
                    if(dp[i][id]>0)
                    {
                        if(minlength==0||minlength>=dp[i][id])
                        {
                            minlength=dp[i][id];
                            minfloorid[i]=j;
                        }
                        flag=0;
                        startid=j;
                    }
                    continue;
                }
                if(dp[i][id]>0&&startid==j-1)
                {
                    if(minlength==0||minlength>=dp[i][id])
                    {
                        minlength=dp[i][id];
                        minfloorid[i]=j;
                    }
                    startid=j;
                }
                else if(dp[i][id]>0)
                {
                    if(minlength==0||minlength>=dp[i][id])
                    {
                        minlength=dp[i][id];
                        minfloorid[i]=j;
                    }
                    double delta=(dp[i][id]-dp[i][startid*size+startid])/(j-startid);
                    int k;
                    for(k=startid+1;k<j;k++)
                    {
                        dp[i][k*size+k]=dp[i][id]-(j-k)*delta;
                    }
                    startid=j;
                }
            }
            dp[i].back()=0;
            //O(M^2)
            for(j=minfloorid[i]+1;j<size;j++)
            {
                int k;
                for(k=j-1;k>=minfloorid[i];k--)
                {
                    int lid=k*size+(j-1);
                    int did=(k+1)*size+j;
                    int id=k*size+j;
                    double lvalue=dp[i][lid];
                    double dvalue=dp[i][did];
                    if(lvalue>0&&dvalue>0)
                    {
                        dp[i][id]=lvalue<dvalue?lvalue:dvalue;
                    }
                    else if(lvalue>0)
                    {
                        dp[i][id]=lvalue;
                    }
                    else if(dvalue>0)
                    {
                        dp[i][id]=dvalue;
                    }
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

    double c=cos(rotation);
    double s=sin(rotation);

#pragma omp parallel for \
    default(shared) \
    schedule(dynamic,10)
    for(int i=0;i<beamnum;i++)
    {
        int floorid=minfloorid[i];
        int ceilingid=size-1;
        int id=floorid*size+ceilingid;
        virtualScan[i]=dp[i][id];
        if(virtualScan[i]<=0)
        {
            continue;
        }
        bool jumpflag=1;
        while(floorid<ceilingid)
        {
            id=floorid*size+ceilingid;
            if(dp[i][id]>0)
            {
                if(dp[i][id+size]>0)
                {
                    if(jumpflag&&dp[i][id+size]-dp[i][id]<delta)
                    {
                        virtualScan[i]=dp[i][id];
                        pathfloor[i].push_back(floorid);
                        pathceiling[i].push_back(ceilingid);
                        floorid++;
                        if(minfloor+floorid*step>dp[i][id]*s+maxFloor*c)
                        {
                            break;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    jumpflag=0;
                    if(dp[i][id+size]-dp[i][id]>=delta)
                    {
                        virtualScan[i]=dp[i][id];
                        pathfloor[i].push_back(floorid);
                        pathceiling[i].push_back(ceilingid);
                        floorid++;
                    }
                    else
                    {
                        double tmpdis=dp[i][id];
                        int j;
                        for(j=ceilingid-1;j>=floorid;j--)
                        {
                            id=floorid*size+j;
                            if(dp[i][id+size]-dp[i][id]>=delta)
                            {
                                virtualScan[i]=dp[i][id];
                                pathfloor[i].push_back(floorid);
                                pathceiling[i].push_back(ceilingid);
                                ceilingid=j;
                                pathfloor[i].push_back(floorid);
                                pathceiling[i].push_back(ceilingid);
                                break;
                            }
                            else if(minfloor+(j+1)*step<dp[i][id]*s+minCeiling*c)
                            {
                                j=floorid;
                            }
                        }
                        if(j<floorid)
                        {
                            virtualScan[i]=tmpdis;
                            pathfloor[i].push_back(floorid);
                            pathceiling[i].push_back(ceilingid);
                            floorid++;
                            break;
                        }
                    }
                }
                else
                {
                    pathfloor[i].push_back(floorid);
                    pathceiling[i].push_back(ceilingid);
                    floorid++;
                    virtualScan[i]=dp[i][id+size];
                    pathfloor[i].push_back(floorid);
                    pathceiling[i].push_back(ceilingid);
                    floorid++;
                    break;
                }
            }
            else
            {
                virtualScan[i]=dp[i][id];
                pathfloor[i].push_back(floorid);
                pathceiling[i].push_back(ceilingid);
                floorid++;
                break;
            }
        }
        heights[i]=minfloor+(floorid-1+0.5)*step;
    }
}

void VirtualScan::getLowerVirtualScan(double theta, double maxFloor, double minCeiling, QVector<double> &virtualScan, QVector<double> &heights)
{

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
