#include "modelbasedtracking.h"

#define PI 3.141592654

int ParticleDataBase::geodim=0;
QVector<double> ParticleDataBase::geolb(0);
QVector<double> ParticleDataBase::geoub(0);
void * ParticleDataBase::measuredata=NULL;
uint * ParticleDataBase::deltaseconds=NULL;

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

void ParticleDataBase::setMeasureData(void *measureData)
{
    assert(measureData!=NULL);
    measuredata=measureData;
}

void ParticleDataBase::setDeltaSeconds(uint *deltaSeconds)
{
    assert (deltaSeconds!=NULL);
    deltaseconds=deltaSeconds;
}

ParticleDataBase::ParticleDataBase(Eigen::Vector3d & initialPosition, Eigen::Vector3d & initialOrientation, Eigen::Vector3d & initialVelocity, QVector<double> & initialGeometry)
{
    position=initialPosition;
    orientation=initialOrientation;
    velocity=initialVelocity;

    int i;
    for(i=0;i<3;i++)
    {
        position(i)=mrpt::random::randomGenerator.drawUniform(position(i)-posuni,position(i)+posuni);
        orientation(i)=mrpt::random::randomGenerator.drawUniform(orientation(i)-posuni,orientation(i)+oriuni);
        velocity(i)=mrpt::random::randomGenerator.drawUniform(velocity(i)-posuni,velocity(i)+veluni);
    }

    assert(geodim>0);
    opt=new nlopt::opt(nlopt::LN_COBYLA,geodim);
    assert(opt!=NULL);
    opt->set_lower_bounds(geolb.toStdVector());
    opt->set_upper_bounds(geoub.toStdVector());
    opt->set_xtol_abs(1e-6);
    opt->set_max_objective(geometryEvaluationFunc,(void *)this);
    optscore=0;

    assert(initialGeometry.size()==geodim);
    mu=initialGeometry;
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

void ParticleDataBase::calculateRotationMatrix()
{
    rotationmatrix=
            Eigen::AngleAxisd(orientation(2),Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(orientation(1),Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(orientation(0),Eigen::Vector3d::UnitX());
}

void ParticleDataBase::motionUpdate()
{
    calculateRotationMatrix();
    position=position+rotationmatrix*((*deltaseconds)*velocity);
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

double ParticleDataBase::updateParticle()
{
    motionUpdate();
    estimateGeometryExpectation();
    estimateGeometryDerivation();
    return calculateWeight();
}
