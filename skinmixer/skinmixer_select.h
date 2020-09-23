#ifndef SKINMIXER_SELECT_H
#define SKINMIXER_SELECT_H

#include <vector>

#include "skinmixer/skinmixer_utilities.h"

namespace skinmixer {

template<class Model>
void computeRemoveSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity,
        std::vector<float>& vertexSelectValue,
        std::vector<bool>& jointSelectValue);

template<class Model>
void computeDetachSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity,
        std::vector<float>& vertexSelectValue,
        std::vector<bool>& jointSelectValue);


}

#include "skinmixer_select.cpp"

#endif // SKINMIXER_SELECT_H
