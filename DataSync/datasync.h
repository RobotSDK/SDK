#ifndef DATASYNC_H
#define DATASYNC_H

#include<qqueue.h>
#include<QTime>
#include<boost/shared_ptr.hpp>

template<class DataType1, class DataType2>
class DataSync
{
public:
    DataSync();
protected:
    QQueue<DataType1> buffer1;
    QQueue<QTime> timestamp1;
    QQueue<DataType2> buffer2;
    QQueue<QTime> timestamp2;
public:
    void addData1(DataType1 data, QTime timestamp);
    void addData2(DataType2 data, QTime timestamp);
    bool getSyncData(DataType1 & data1, DataType2 & data2);
    void clear();
};

template<class DataType1, class DataType2>
DataSync<DataType1,DataType2>::DataSync()
{
    buffer1.clear();
    buffer2.clear();
}

template<class DataType1, class DataType2>
void DataSync<DataType1,DataType2>::addData1(DataType1 data, QTime timestamp)
{
    buffer1.push_back(data);
    timestamp1.push_back(timestamp);
}

template<class DataType1, class DataType2>
void DataSync<DataType1,DataType2>::addData2(DataType2 data, QTime timestamp)
{
    buffer2.push_back(data);
    timestamp2.push_back(timestamp);
}

template<class DataType1, class DataType2>
bool DataSync<DataType1,DataType2>::getSyncData(DataType1 & data1, DataType2 & data2)
{
    int i=0,n=buffer1.size();
    int j=0,m=buffer2.size();
    if(n==0||m==0)
    {
        return 0;
    }
    int delta=0;
    while(i<n&&j<m)
    {
        QTime stamp1=timestamp1[i];
        QTime stamp2=timestamp2[j];
        int tmpdelta=stamp1.msecsTo(stamp2);
        if(tmpdelta==0)
        {
            data1=buffer1[i];
            buffer1.erase(buffer1.begin(),buffer1.begin()+i+1);
            timestamp1.erase(timestamp1.begin(),timestamp1.begin()+i+1);
            data2=buffer2[j];
            buffer2.erase(buffer2.begin(),buffer2.begin()+j+1);
            timestamp2.erase(timestamp2.begin(),timestamp2.begin()+j+1);
            return 1;
        }
        else
        {
            if(delta==0)
            {
                delta=tmpdelta;
                if(delta>0)
                {
                    i++;
                }
                else
                {
                    j++;
                }
            }
            else
            {
                if(abs(tmpdelta)<abs(delta))
                {
                    delta=tmpdelta;
                    if(delta>0)
                    {
                        i++;
                    }
                    else
                    {
                        j++;
                    }
                }
                else
                {
                    if(delta>0)
                    {
                        i--;
                    }
                    else
                    {
                        j--;
                    }
                    break;
                }
            }
        }
    }
    if(i==n)
    {
        buffer1.erase(buffer1.begin(),buffer1.begin()+i-1);
        timestamp1.erase(timestamp1.begin(),timestamp1.begin()+i-1);
        buffer2.erase(buffer2.begin(),buffer2.begin()+j);
        timestamp2.erase(timestamp2.begin(),timestamp2.begin()+j);
        return 0;
    }
    if(j==m)
    {
        buffer1.erase(buffer1.begin(),buffer1.begin()+i);
        timestamp1.erase(timestamp1.begin(),timestamp1.begin()+i);
        buffer2.erase(buffer2.begin(),buffer2.begin()+j-1);
        timestamp2.erase(timestamp2.begin(),timestamp2.begin()+j-1);
        return 0;
    }
    data1=buffer1[i];
    buffer1.erase(buffer1.begin(),buffer1.begin()+i+1);
    timestamp1.erase(timestamp1.begin(),timestamp1.begin()+i+1);
    data2=buffer2[j];
    buffer2.erase(buffer2.begin(),buffer2.begin()+j+1);
    timestamp2.erase(timestamp2.begin(),timestamp2.begin()+j+1);
    return 1;
}

template<class DataType1, class DataType2>
void DataSync<DataType1,DataType2>::clear()
{
    buffer1.clear();
    timestamp1.clear();
    buffer2.clear();
    timestamp2.clear();
}

#endif // DATASYNC_H
