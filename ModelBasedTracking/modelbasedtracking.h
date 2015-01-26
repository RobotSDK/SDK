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

#define GEOMETRYSAMPLENUM 100

class ParticleDataBase
{
protected:
    static int geodim;
    static QVector<double> geolb;
    static QVector<double> geoub;
    static void * measuredata;
    static uint * deltaseconds;
public:
    static double posuni;
    static double posder;
    static double oriuni;
    static double orider;
    static double veluni;
    static double velder;
protected:
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    Eigen::Matrix3d rotationmatrix;
    Eigen::Vector3d velocity;
    nlopt::opt * opt;
    double optscore;
    QVector<double> mu;
    QVector<double> geosigma;
    QVector<double> sigma;
    QVector<double> premu;
    QVector<double> presigma;
    bool initflag;
public:
    static void setGeometryRange(QVector<double> & geometryLowerBounds, QVector<double> & geometryUpperBounds);
    static void setMeasureData(void * measureData);
    static void setDeltaSeconds(uint * deltaSeconds);
    static void setMotionParams();
public:
    ParticleDataBase(Eigen::Vector3d & initialPosition, Eigen::Vector3d & initialOrientation, Eigen::Vector3d & initialVelocity, QVector<double> & initialGeometry);
    virtual ~ParticleDataBase();
protected:
    static double geometryEvaluationFunc(const std::vector<double> & G, std::vector<double> & grad, void * particleDataBase);
    virtual double geometryEvaluation(const QVector<double> & G)=0;
protected:
    void calculateRotationMatrix();
protected:
    void motionUpdate();
    void estimateGeometryExpectation();
    void estimateGeometryDerivation();
    double calculateWeight();
public:
    double updateParticle();
public:
    Eigen::Vector3d getPosition();
    Eigen::Vector3d getOrientation();
    Eigen::Vector3d getVelocity();
    QVector<double> getGeometry();
};

#define ModeBasedTrackingClass(ParticleDataType) \
class ModelBasedTracking_##ParticleDataType : \
        public mrpt::bayes::CParticleFilterData<ParticleDataType>, \
        public mrpt::bayes::CParticleFilterDataImpl<ModelBasedTracking,mrpt::bayes::CParticleFilterData<ParticleDataType>::CParticleList>{ \
protected: \
    void prediction_and_update_pfStandardProposal(const obs::CActionCollection *action,const obs::CSensoryFrame *observation,const CParticleFilter::TParticleFilterOptions &PF_options) \
    {size_t i,n=m_particles.size();for(i=0;i<n;i++){m_particles[i].log_w=m_particles[i].d->updateParticle();}} \
public: \
    void setParticleNumber(size_t particleNumber) \
    {clearParticles();m_particles.resize(particleNumber);} \
    void initializeParticles(Eigen::Vector3d & initialPosition, Eigen::Vector3d & initialOrientation, Eigen::Vector3d & initialVelocity, QVector<double> & initialGeometry) \
    {int i,n=m_particles.size();for(i=0;i<n;i++){m_particles[i].d=new ParticleDataType(initialPosition,initialOrientation,initialVelocity,priorGeometry);m_particles[i].log_w=0;}} \
    void estimateParticles(Eigen::Vector3d & estimatePosition, Eigen::Vector3d & estimateOrientation, Eigen::Vector3d & estimateVelocity, QVector<double> & estimateGeometry) \
    {estimatePosition=Eigen::Vector3d::Zero();estimateOrientation=Eigen::Vector3d::Zero();estimateVelocity=Eigen::Vector3d::Zero();estimateGeometry.fill(0,geodim);double sum=0; \
    int i,n=m_particles.size();for(i=0;i<n;i++){double weight=exp(m_particles[i].log_w);estimatePosition+=weight*m_particles[i].d->getPosition();estimateOrientation+=weight*m_particles[i].d->getOrientation();estimateVelocity+=weight*m_particles[i].d->getVelocity(); \
    int j;QVector<double> tmpgeometry=m_particles[i].d->getGeometry();for(j=0;j<geodim;j++){estimateGeometry[j]+=weight*tmpgeometry[j];}sum+=weight;}assert(sum>0); \
    for(i=0;i<n;i++){estimatePosition/=sum;estimateOrientation/=sum;estimateVelocity/=sum;int j;for(j=0;j<geodim;j++){estimateGeometry[j]/=sum;}}}};

#define ModelBasedTracking(ParticleDataType) ModelBasedTracking_##ParticleDataType

#endif // MODELBASEDTRACKING_H
