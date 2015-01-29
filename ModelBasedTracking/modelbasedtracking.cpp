#include "modelbasedtracking.h"

#define PI 3.141592654

int ParticleDataBase::geodim=0;
QVector<double> ParticleDataBase::geolb(0);
QVector<double> ParticleDataBase::geoub(0);
void * ParticleDataBase::measuredata=NULL;
int * ParticleDataBase::deltamsecs=NULL;

double ParticleDataBase::posuni=1;
double ParticleDataBase::posder=1;
double ParticleDataBase::oriuni=(PI/180.0)*10;
double ParticleDataBase::orider=(PI/180.0)*10;
double ParticleDataBase::veluni=1;
double ParticleDataBase::velder=1;

void ParticleDataBase::setGeometryRange(QVector<double> & geometryLowerBounds, QVector<double> & geometryUpperBounds)
{
    assert(geometryLowerBounds.size()>0&&geometryLowerBounds.size()==geometryUpperBounds.size());
    geodim=geometryLowerBounds.size();
    geolb=geometryLowerBounds;
    geoub=geometryUpperBounds;
}

void ParticleDataBase::linkMeasureData(void *measureData)
{
    assert(measureData!=NULL);
    measuredata=measureData;
}

void ParticleDataBase::linkDeltaSeconds(int *deltaMsecs)
{
    assert (deltaMsecs!=NULL);
    deltamsecs=deltaMsecs;
}

ParticleDataBase::ParticleDataBase()
{
    position=Eigen::Vector3d::Zero();
    orientation=Eigen::Vector3d::Zero();
    velocity=Eigen::Vector3d::Zero();

    assert(geodim>0);
    opt=new nlopt::opt(nlopt::LN_COBYLA,geodim);
    assert(opt!=NULL);
    opt->set_lower_bounds(geolb.toStdVector());
    opt->set_upper_bounds(geoub.toStdVector());
    opt->set_xtol_abs(1e-6);
    opt->set_max_objective(geometryEvaluationFunc,(void *)this);
    optscore=0;

    mu.fill(0,geodim);
    geosigma.fill(0,geodim);
    sigma.fill(0,geodim);
    premu.fill(0,geodim);
    presigma.fill(0,geodim);
    initflag=0;
}

ParticleDataBase::~ParticleDataBase()
{
    if(opt!=NULL)
    {
        delete opt;
        opt=NULL;
    }
}

double ParticleDataBase::geometryEvaluationFunc(const std::vector<double> & G, std::vector<double> & grad, void * particleDataBase)
{
    ParticleDataBase * particle=static_cast<ParticleDataBase *>(particleDataBase);
    return particle->geometryEvaluation(QVector<double>::fromStdVector(G));
}

double ParticleDataBase::geometryEvaluation(const QVector<double> &G)
{
    return 0;
}

void ParticleDataBase::calculateRotationMatrix()
{
    rotationmatrix=
            Eigen::AngleAxisd(orientation(2),Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(orientation(1),Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(orientation(0),Eigen::Vector3d::UnitX());
}

void ParticleDataBase::motionUpdate()
{
    assert(deltamsecs!=NULL);
    if(*deltamsecs<=0)
    {
        return;
    }
    calculateRotationMatrix();
    position=position+rotationmatrix*((*deltamsecs)/1000.0*velocity);
    int i;
    for(i=0;i<3;i++)
    {
        position(i)+=posder*mrpt::random::randomGenerator.drawGaussian1D_normalized();
        orientation(i)+=orider*mrpt::random::randomGenerator.drawGaussian1D_normalized();
        velocity(i)+=velder*mrpt::random::randomGenerator.drawGaussian1D_normalized();
    }
}

void ParticleDataBase::estimateGeometryExpectation()
{
    premu=mu;
    std::vector<double> x=mu.toStdVector();
    opt->optimize(x,optscore);
    mu=QVector<double>::fromStdVector(x);
}

void ParticleDataBase::estimateGeometryDerivation()
{
    presigma=sigma;
    int i;
    for(i=0;i<geodim;i++)
    {
        geosigma[i]=0;
        QVector<double> tmpG=mu;
        double density=(geoub[i]-geolb[i])/GEOMETRYSAMPLENUM;
        QVector<double> score(GEOMETRYSAMPLENUM+1);
        double sum=0;
        int j;
        for(j=0;j<=GEOMETRYSAMPLENUM;j++)
        {
            tmpG[i]=geoub[i]+j*density;
            score[j]=geometryEvaluation(tmpG);
            sum+=score[j];
            geosigma[i]+=pow(tmpG[i]-mu[i],2.0)*score[j];
        }
        geosigma[i]/=sum;
        if(initflag)
        {
            sigma[i]=(sigma[i]*geosigma[i])/(sigma[i]+geosigma[i]);
        }
        else
        {
            sigma[i]=geosigma[i];
        }
    }
}

double ParticleDataBase::calculateWeight()
{
    if(initflag)
    {
        double w=log(optscore);
        int i;
        for(i=0;i<geodim;i++)
        {
            w+=log(sqrt(sigma[i]))-log(sqrt(presigma[i]));
            w+=-0.5*pow(mu[i]-premu[i],2.0)/(geosigma[i]+presigma[i]);
        }
        return w;
    }
    else
    {
        initflag=1;
        return 0;
    }
}

void ParticleDataBase::initialParticle(ParticleDataBase * particleData)
{
    position=particleData->position;
    orientation=particleData->orientation;
    velocity=particleData->velocity;
    mu=particleData->mu;

    int i;
    for(i=0;i<3;i++)
    {
        position(i)=mrpt::random::randomGenerator.drawUniform(position(i)-posuni,position(i)+posuni);
        orientation(i)=mrpt::random::randomGenerator.drawUniform(orientation(i)-posuni,orientation(i)+oriuni);
        velocity(i)=mrpt::random::randomGenerator.drawUniform(velocity(i)-posuni,velocity(i)+veluni);
    }

    initflag=0;
}

double ParticleDataBase::updateParticle()
{
    motionUpdate();
    estimateGeometryExpectation();
    estimateGeometryDerivation();
    return calculateWeight();
}

void ModelBasedTracking::prediction_and_update_pfStandardProposal(
        const mrpt::obs::CActionCollection *action,
        const mrpt::obs::CSensoryFrame *observation,
        const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options)
{
    size_t i,n=m_particles.size();
    for(i=0;i<n;i++)
    {
        m_particles[i].log_w=m_particles[i].d->updateParticle();
    }
}

void ModelBasedTracking::setParticleNumber(size_t particleNumber)
{
    assert(particleNumber>0);
    clearParticles();
    m_particles.resize(particleNumber);
}

void ModelBasedTracking::estimateParticles(ParticleDataBase *particleEstimateData)
{
    assert(particleEstimateData!=NULL);
    particleEstimateData->position=Eigen::Vector3d::Zero();
    particleEstimateData->orientation=Eigen::Vector3d::Zero();
    particleEstimateData->velocity=Eigen::Vector3d::Zero();
    particleEstimateData->mu.fill(0);
    particleEstimateData->sigma.fill(0);
    double sum=0;
    int i,n=m_particles.size();
    for(i=0;i<n;i++)
    {
        double weight=exp(m_particles[i].log_w);
        particleEstimateData->position+=weight*m_particles[i].d->position;
        particleEstimateData->orientation+=weight*m_particles[i].d->orientation;
        particleEstimateData->velocity+=weight*m_particles[i].d->velocity;
        int j,m=particleEstimateData->mu.size();
        for(j=0;j<m;j++)
        {
            particleEstimateData->mu[j]+=weight*(m_particles[i].d->mu[j]);
            particleEstimateData->sigma[j]+=weight*(m_particles[i].d->sigma[j]);
        }
        sum+=weight;
    }
    assert(sum>0);
    particleEstimateData->position/=sum;
    particleEstimateData->orientation/=sum;
    particleEstimateData->velocity/=sum;
    int j,m=particleEstimateData->mu.size();
    for(j=0;j<m;j++)
    {
        particleEstimateData->mu[j]/=sum;
        particleEstimateData->sigma[j]/=sum;
    }
}
