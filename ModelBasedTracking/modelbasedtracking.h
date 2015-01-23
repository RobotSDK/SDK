#ifndef MODELBASEDTRACKING_H
#define MODELBASEDTRACKING_H

#include<mrpt/bayes/CParticleFilter.h>
#include<mrpt/bayes/CParticleFilterCapable.h>
#include<mrpt/bayes/CParticleFilterData.h>
#include<mrpt/bayes/CProbabilityParticle.h>

#include<mrpt/random/RandomGenerators.h>

#include<eigen3/Eigen/Dense>

template<int G_Dim>
class ParticleDataType
{
public:
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
    Eigen::Vector3d velocity;
    Eigen::VectorXd mu;
    Eigen::MatrixXd sigma;
public:
    ParticleDataType();
};

template<int G_Dim>
ParticleDataType<G_Dim>::ParticleDataType()
{
    position=Eigen::Vector3d::Zero();
    orientation=Eigen::Vector3d::Zero();
    velocity=Eigen::Vector3d::Zero();
    mu=Eigen::VectorXd::Zero(G_Dim);
    sigma=Eigen::MatrixXd::Zero(G_Dim,G_Dim);
}

template<int G_Dim>
class ModelBasedTracking :
        public mrpt::bayes::CParticleFilterData<ParticleDataType<G_Dim>>,
        public mrpt::bayes::CParticleFilterDataImpl<ModelBasedTracking,mrpt::bayes::CParticleFilterData<ParticleDataType<G_Dim>>::CParticleList>
{
protected:

public:
    ModelBasedTracking();
};

#endif // MODELBASEDTRACKING_H
