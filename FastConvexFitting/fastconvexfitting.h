#ifndef FASTCONVEXFITTING_H
#define FASTCONVEXFITTING_H

#include<Eigen/Dense>
#include<Eigen/Sparse>

#include<QVector>
#include<QList>

struct Edge
{
    Eigen::Vector2d startcorner;
    Eigen::Vector2d endcorner;
};

struct Geometry
{
    QVector<Edge> edges;
    QVector<Edge> globaledges;
    QVector<double> geometry;
    double score;
};

class FastConvexFitting
{
public:
    FastConvexFitting(QVector<double> & geometryLowerBound, QVector<double> & geometryUpperBound, QVector<double> & geometryStep);
protected:
    Geometry G;
    QVector<double> lb;
    QVector<double> ub;
    QVector<double> step;

    Eigen::Vector2d position;
    Eigen::Matrix2d orientation;
    Eigen::Vector2d centerposition;

    int beamsnum;
    QVector<double> beams;
    QVector<Eigen::Vector2d> points;

    bool initflag;
    QVector<Geometry> configurations;
protected:
    virtual bool getConfigurations()=0;
    virtual bool getBeamRange(Geometry & configuration, QVector<int> & edgeID, QVector<int> & startID, QVector<int> & endID)=0;
    virtual double getGain(double & alpha, double & beta, double & beam);
public:
    void updatePosition(double x, double y);
    void updateOrientation(double theta);
    void updateScanBeams(QVector<double> & scanBeams);
    bool getEvaluation(Geometry & geometry);
    bool getFitting(Geometry & geometry);
};

class FastRectangleFitting : public FastConvexFitting
{
public:
    FastRectangleFitting(QVector<double> & geometryLowerBound, QVector<double> & geometryUpperBound, QVector<double> & geometryStep);
protected:
    bool getConfigurations();
    bool getBeamRange(Geometry &configuration, QVector<int> &edgeID, QVector<int> &startID, QVector<int> &endID);
};

#endif // FASTCONVEXFITTING_H
