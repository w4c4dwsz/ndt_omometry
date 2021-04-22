#include "utility.h"

class ndt_set
{
private:
    float MaximumIterations_;
    float TransformationEpsilon_;
    float StepSize_;
    float Resolution_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> * ndt_;
public:
    ndt_set(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> * ndt):
        MaximumIterations_(35),
        TransformationEpsilon_(0.01),
        StepSize_(0.1),
        Resolution_(1),
        ndt_(ndt){}

    ~ndt_set(){};

    void setMaximumIterations(float MaximumIterations)
    {
        MaximumIterations_ = MaximumIterations;
    }

    void setTransformationEpsilon(float TransformationEpsilon)
    {
        TransformationEpsilon_ = TransformationEpsilon;
    }
    
    void setStepSize(float stepsize)
    {
        StepSize_ = stepsize;
    }

    void setResolution(float Resolution)
    {
        Resolution_ = Resolution;
    }

    void init()
    {
        ndt_->setMaximumIterations(MaximumIterations_);
        ndt_->setTransformationEpsilon(TransformationEpsilon_);
        ndt_->setResolution(Resolution_);
    }
};

