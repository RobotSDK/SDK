#ifndef PARTICLEFILTERDEF_H
#define PARTICLEFILTERDEF_H

#include<vector>

#define StateValueType float
#define BlockThreadNum 256
#define GridBlockNum(ParticleNum) (ParticleNum+BlockThreadNum-1)/BlockThreadNum
#define GetThreadID(id,ParticleNum) int id=blockDim.x*blockIdx.x+threadIdx.x;if(id>=ParticleNum){return;}
#define CUDAFREE(pointer) if(pointer!=NULL){cudaFree(pointer);pointer=NULL;}

#define STATE_UNION_DEF(StateNum, ...) \
    typedef union {StateValueType data[StateNum];struct{StateValueType __VA_ARGS__;};}

#define STATE_TYPE(PFName) PFName##_State
#define STATE_DEF(PFName, StateNum, ...) \
    STATE_UNION_DEF(StateNum, __VA_ARGS__) STATE_TYPE(PFName);

#define SAMPLESTATE_TYPE(PFName) PFName##_SampleState
#define SAMPLESTATE_DEF(PFName, StateNum, ...) \
    STATE_UNION_DEF(StateNum, __VA_ARGS__) SAMPLESTATE_TYPE(PFName);

#define RANDOMSTATE_TYPE(PFName) PFName##_RandomState
#define RANDOMSTATE_DEF(PFName, StateNum, ...) \
    STATE_UNION_DEF(StateNum, __VA_ARGS__) RANDOMSTATE_TYPE(PFName);

#define STATE_NUM(StateTypeName) \
    sizeof(StateTypeName)/sizeof(StateValueType)

#define MEASUREDATA_TYPE(PFName) PFName##_MeasureData
#define MEASUREDATA_DEF(PFName, MeasureDataType) \
    typedef MeasureDataType MEASUREDATA_TYPE(PFName);

#define PF_DEF(PFName) \
    typedef ParticleBase<STATE_TYPE(PFName), SAMPLESTATE_TYPE(PFName), RANDOMSTATE_TYPE(PFName)> PFName##_ParticleBase; \
    class PFName##_Particle : public PFName##_ParticleBase \
    { \
    public: \
        __host__ __device__ \
        void initialize(STATE_TYPE(PFName) & initialState, SAMPLESTATE_TYPE(PFName) & randomOffset); \
        __host__ __device__ \
        void randomnize(RANDOMSTATE_TYPE(PFName) & randomOffset); \
        __host__ __device__ \
        void transform(EgoTransform & transform); \
        __host__ __device__ \
        void update(int & deltaMsec); \
    }; \
    typedef ParticleMeasureBase<STATE_TYPE(PFName),MEASUREDATA_TYPE(PFName)> PFName##_MeasureBase; \
    class PFName##_Measure : public PFName##_MeasureBase \
    { \
    public: \
        __host__ __device__ \
        float particleMeasure(STATE_TYPE(PFName) & state, MEASUREDATA_TYPE(PFName) & measureData); \
    }; \
    typedef ParticleFilterBase<STATE_TYPE(PFName),PFName##_Particle,MEASUREDATA_TYPE(PFName),PFName##_Measure> PFName##_Base; \
    class PFName : public PFName##_Base \
    { \
    public: \
        void measureParticles(MEASUREDATA_TYPE(PFName) & measureData); \
    };

#define PARTICLE_INITIALIZE_FUNC(PFName) \
    __host__ __device__ void PFName##_Particle::initialize(STATE_TYPE(PFName) & initialState, SAMPLESTATE_TYPE(PFName) & randomOffset)

#define PARTICLE_RANDOMNIZE_FUNC(PFName) \
    __host__ __device__ void PFName##_Particle::randomnize(RANDOMSTATE_TYPE(PFName) & randomOffset)

#define PARTICLE_UPDATE_FUNC(PFName, deltaMsec) \
    __host__ __device__ void PFName##_Particle::update(int & deltaMsec)

#define PARTICLE_TRANSFORM_FUNC(PFName, transform) \
    __host__ __device__ void PFName##_Particle::transform(EgoTransform & transform)

#define PARTICLE_MEASURE_FUNC(PFName) \
    __host__ __device__ float PFName##_Measure::particleMeasure(STATE_TYPE(PFName) & particleState, MEASUREDATA_TYPE(PFName) & measureData)

#define PARTICLE_FILTER_MEASURE_FUNC(PFName) \
    void PFName::measureParticles(MEASUREDATA_TYPE(PFName) & measureData)

#define PARTICLE_FILTER_MEASURE_DEFAULT_FUNC(PFName) \
    PARTICLE_FILTER_MEASURE_FUNC(PFName) \
    { \
        PFName##_Base::measureParticles(measureData); \
    }

#define PARTICLE_FILTER_INTERACT_FUNCS_DECL(PFName) \
    extern "C" void PF_##PFName##_initialParticleFilter(int particleNum, SAMPLESTATE_TYPE(PFName) & sampleMin, SAMPLESTATE_TYPE(PFName) & sampleMax, RANDOMSTATE_TYPE(PFName) & randomMean, RANDOMSTATE_TYPE(PFName) & randomSigma); \
    extern "C" void PF_##PFName##_addObjectState(const int objectID, const STATE_TYPE(PFName) & objectState); \
    extern "C" void PF_##PFName##_addObjectStates(const std::vector<int> & objectID, const std::vector<STATE_TYPE(PFName)> & objectState); \
    extern "C" void PF_##PFName##_advanceParticleFilter(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData); \
    extern "C" void PF_##PFName##_advanceParticleFilter2D(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData, StateValueType dx=0, StateValueType dy=0, StateValueType theta1=0, StateValueType theta2=0); \
    extern "C" void PF_##PFName##_advanceParticleFilter3D(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData, StateValueType * transformMatrix=NULL); \
    extern "C" void PF_##PFName##_removeParticles(float minWeightThreshold); \
    extern "C" bool PF_##PFName##_estimateObject(int objectID, STATE_TYPE(PFName) & objectState, float & normalizer); \
    extern "C" void PF_##PFName##_estimateObjects(std::vector<int> & objectsID, std::vector<STATE_TYPE(PFName)> & objectsState, std::vector<float> & normalizer); \
    extern "C" void PF_##PFName##_clear(); \
    extern "C" void PF_##PFName##_randomnizeParticles(); \
    extern "C" void PF_##PFName##_updateParticles(int deltaMsec); \
    extern "C" void PF_##PFName##_transformParticles2D(StateValueType dx, StateValueType dy, StateValueType theta1, StateValueType theta2); \
    extern "C" void PF_##PFName##_transformParticles3D(StateValueType * transformMatrix); \
    extern "C" void PF_##PFName##_measureParticles(MEASUREDATA_TYPE(PFName) & measureData); \
    extern "C" void PF_##PFName##_resampleParticles(); \
    extern "C" void PF_##PFName##_collectParticles(); \
    extern "C" void PF_##PFName##_debugParticleFilterGetParticles(std::vector< std::vector<STATE_TYPE(PFName)> > & particleState, std::vector< std::vector<float> > & particleWeight); \
    extern "C" void PF_##PFName##_debugParticleFilterSetParticles(std::vector< std::vector<STATE_TYPE(PFName)> > & particleState, std::vector< std::vector<float> > & particleWeight);

#define PARTICLE_INSTANCE(PFName) PFName##_particlefilter
#define PARTICLE_FILTER_INTERACT_FUNCS(PFName) \
    PFName PARTICLE_INSTANCE(PFName); \
    extern "C" void PF_##PFName##_initialParticleFilter(int particleNum, SAMPLESTATE_TYPE(PFName) & sampleMin, SAMPLESTATE_TYPE(PFName) & sampleMax, RANDOMSTATE_TYPE(PFName) & randomMean, RANDOMSTATE_TYPE(PFName) & randomSigma) \
    { \
        PARTICLE_INSTANCE(PFName).initialParticleFilter(particleNum,sampleMin,sampleMax,randomMean,randomSigma); \
    } \
    extern "C" void PF_##PFName##_addObjectState(const int objectID, const STATE_TYPE(PFName) & objectState) \
    { \
        PARTICLE_INSTANCE(PFName).addObjectState(objectID,objectState); \
    } \
    extern "C" void PF_##PFName##_addObjectStates(const std::vector<int> & objectID, const std::vector<STATE_TYPE(PFName)> & objectState) \
    { \
        PARTICLE_INSTANCE(PFName).addObjectState(objectID,objectState); \
    } \
    extern "C" void PF_##PFName##_advanceParticleFilter(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData) \
    { \
        PARTICLE_INSTANCE(PFName).randomnizeParticles(); \
        PARTICLE_INSTANCE(PFName).updateParticles(deltaMsec); \
        PARTICLE_INSTANCE(PFName).measureParticles(measureData); \
        PARTICLE_INSTANCE(PFName).resampleParticles(); \
        PARTICLE_INSTANCE(PFName).collectParticles(); \
        cudaDeviceSynchronize(); \
    } \
    extern "C" void PF_##PFName##_advanceParticleFilter2D(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData, StateValueType dx, StateValueType dy, StateValueType theta1, StateValueType theta2) \
    { \
        EgoTransform transform(dx,dy,theta1,theta2); \
        PARTICLE_INSTANCE(PFName).randomnizeParticles(); \
        PARTICLE_INSTANCE(PFName).updateParticles(deltaMsec); \
        PARTICLE_INSTANCE(PFName).transformParticles(transform); \
        PARTICLE_INSTANCE(PFName).measureParticles(measureData); \
        PARTICLE_INSTANCE(PFName).resampleParticles(); \
        PARTICLE_INSTANCE(PFName).collectParticles(); \
        cudaDeviceSynchronize(); \
    } \
    extern "C" void PF_##PFName##_advanceParticleFilter3D(int deltaMsec, MEASUREDATA_TYPE(PFName) & measureData, StateValueType * transformMatrix) \
    { \
        EgoTransform transform(transformMatrix); \
        PARTICLE_INSTANCE(PFName).randomnizeParticles(); \
        PARTICLE_INSTANCE(PFName).updateParticles(deltaMsec); \
        PARTICLE_INSTANCE(PFName).transformParticles(transform); \
        PARTICLE_INSTANCE(PFName).measureParticles(measureData); \
        PARTICLE_INSTANCE(PFName).resampleParticles(); \
        PARTICLE_INSTANCE(PFName).collectParticles(); \
        cudaDeviceSynchronize(); \
    } \
    extern "C" void PF_##PFName##_removeParticles(float minWeightThreshold) \
    { \
        PARTICLE_INSTANCE(PFName).removeParticles(minWeightThreshold); \
    } \
    extern "C" bool PF_##PFName##_estimateObject(int objectID, STATE_TYPE(PFName) & objectState, float & normalizer) \
    { \
        return PARTICLE_INSTANCE(PFName).estimateObjectState(objectID,objectState,normalizer); \
    } \
    extern "C" void PF_##PFName##_estimateObjects(std::vector<int> & objectsID, std::vector<STATE_TYPE(PFName)> & objectsState, std::vector<float> & normalizer) \
    { \
        objectsID=PARTICLE_INSTANCE(PFName).estimateObjectState(objectsState,normalizer); \
    } \
    extern "C" void PF_##PFName##_clear() \
    { \
        PARTICLE_INSTANCE(PFName).clear(); \
    } \
    extern "C" void PF_##PFName##_randomnizeParticles() \
    { \
        PARTICLE_INSTANCE(PFName).randomnizeParticles(); \
    } \
    extern "C" void PF_##PFName##_updateParticles(int deltaMsec) \
    { \
        PARTICLE_INSTANCE(PFName).updateParticles(deltaMsec); \
    } \
    extern "C" void PF_##PFName##_transformParticles2D(StateValueType dx, StateValueType dy, StateValueType theta1, StateValueType theta2) \
    { \
        EgoTransform transform(dx,dy,theta1,theta2); \
        PARTICLE_INSTANCE(PFName).transformParticles(transform); \
    } \
    extern "C" void PF_##PFName##_transformParticles3D(StateValueType * transformMatrix) \
    { \
        EgoTransform transform(transformMatrix); \
        PARTICLE_INSTANCE(PFName).transformParticles(transform); \
    } \
    extern "C" void PF_##PFName##_measureParticles(MEASUREDATA_TYPE(PFName) & measureData) \
    { \
        PARTICLE_INSTANCE(PFName).measureParticles(measureData); \
    } \
    extern "C" void PF_##PFName##_resampleParticles() \
    { \
        PARTICLE_INSTANCE(PFName).resampleParticles(); \
    } \
    extern "C" void PF_##PFName##_collectParticles() \
    { \
        PARTICLE_INSTANCE(PFName).collectParticles(); \
    } \
    extern "C" void PF_##PFName##_debugParticleFilterGetParticles(std::vector< std::vector<STATE_TYPE(PFName)> > & particleState, std::vector< std::vector<float> > & particleWeight) \
    { \
        PARTICLE_INSTANCE(PFName).debugParticleFilterGetParticles(particleState,particleWeight); \
    } \
    extern "C" void PF_##PFName##_debugParticleFilterSetParticles(std::vector< std::vector<STATE_TYPE(PFName)> > & particleState, std::vector< std::vector<float> > & particleWeight) \
    { \
        PARTICLE_INSTANCE(PFName).debugParticleFilterSetParticles(particleState,particleWeight); \
    }

#endif // PARTICLEFILTERDEF_H

