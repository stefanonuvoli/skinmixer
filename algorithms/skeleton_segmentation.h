#ifndef SKELETONSEGMENTATION_H
#define SKELETONSEGMENTATION_H

#include <vector>

#include <nvl/utilities/dummy_variables.h>

namespace skinmixer {

template<class Model>
std::vector<int> skeletonSegmentationMax(
        const Model& model);

template<class Model>
std::vector<int> skeletonSegmentationGraphcut(
        const Model& model,
        double compactness);

template<class Model>
std::vector<int> skeletonBinarySegmentationGraphcut(
        const Model& model,
        double compactness,
        typename Model::Skeleton::JointId jointId,
        std::vector<int>& jointSegmentation = nvl::internal::dummyVectorInt);

}

#include "skeleton_segmentation.cpp"

#endif // SKELETONSEGMENTATION_H
