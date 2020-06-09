#ifndef SKINMIXER_FUZZY_H
#define SKINMIXER_FUZZY_H

#include <vector>

#include "skinmixer/skinmixer_utilities.h"

namespace skinmixer {

template<class Model>
void removeFuzzy(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity);

template<class Model>
void detachFuzzy(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity);


}

#include "skinmixer_fuzzy.cpp"

#endif // SKINMIXER_FUZZY_H
