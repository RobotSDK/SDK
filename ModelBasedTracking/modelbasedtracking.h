#ifndef MODELBASEDTRACKING_H
#define MODELBASEDTRACKING_H

#include<mrpt/bayes/CParticleFilter.h>
#include<mrpt/bayes/CParticleFilterCapable.h>
#include<mrpt/bayes/CParticleFilterData.h>
#include<mrpt/bayes/CProbabilityParticle.h>
#include<mrpt/random/RandomGenerators.h>

#include<nlopt.hpp>
#include<eigen3/Eigen/Dense>

#include<assert.h>
#include<QVector>
#include<QDebug>

#define GEOMETRYSAMPLENUM 100

//class ModelBasedTracking;

class ParticleDataBase
{
protected:
    static int geodim;
    static QVector<double> geolb;
    static QVector<double> geoub;
    static void * measuredata;
    static int * deltamsecs;
public:
    static double posuni;
    static double posder;
    static double oriuni;
    static double orider;
    static double veluni;
    static double velder;
protected:
    nlopt::opt * opt;
    double optscore;
    QVector<double> geosigma;
    QVector<double> premu;
    QVector<double> presigma;
    bool initflag;
public:
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    Eigen::Matrix3d rotationmatrix;
    Eigen::Vector3d velocity;
    QVector<double> mu;
    QVector<double> sigma;
public:
    static void setGeometryRange(QVector<double> & geometryLowerBounds, QVector<double> & geometryUpperBounds);
    static void linkMeasureData(void * measureData);
    static void linkDeltaSeconds(int * deltaMsecs);
public:
    ParticleDataBase();
    virtual ~ParticleDataBase();
protected:
    static double geometryEvaluationFunc(const std::vector<double> & G, std::vector<double> & grad, void * particleDataBase);
    virtual double geometryEvaluation(const QVector<double> & G);
protected:
    void calculateRotationMatrix();
public:
    virtual void motionUpdate();
    void estimateGeometryExpectation();
    void estimateGeometryDerivation();
    double calculateWeight();
public:
    void initialParticle(ParticleDataBase * particleData);
    double updateParticle();
public:
    friend class ModelBasedTracking;
};

class ModelBasedTracking :
        public mrpt::bayes::CParticleFilterData<ParticleDataBase>,
        public mrpt::bayes::CParticleFilterDataImpl<ModelBasedTracking,mrpt::bayes::CParticleFilterData<ParticleDataBase>::CParticleList>
{
protected:
    void prediction_and_update_pfStandardProposal(
            const mrpt::obs::CActionCollection *action,
            const mrpt::obs::CSensoryFrame *observation,
            const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options);
public:
    void setParticleNumber(size_t particleNumber);
    template<class ParticleDataType>
    void initializeParticles(ParticleDataType * particleInitialData)
    {
        int i,n=m_particles.size();
        for(i=0;i<n;i++)
        {
            m_particles[i].d=new ParticleDataType;
            m_particles[i].d->initialParticle(particleInitialData);
            m_particles[i].log_w=0;
        }
    }
    void estimateParticles(ParticleDataBase * particleEstimateData);
};

#endif // MODELBASEDTRACKING_H
