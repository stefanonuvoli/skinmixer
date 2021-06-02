#ifndef SKINMIXER_SELECT_H
#define SKINMIXER_SELECT_H

#include <vector>

#include "skinmixer/skinmixer_utilities.h"

namespace skinmixer {

template<class Model>
void computeSelectValues(
        const Model& model,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const std::vector<double>& jointSelectValue,
        std::vector<double>& vertexSelectValue);

template<class Model>
void computeReplaceSelectValues(
        const Model& model1,
        const Model& model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2,
        std::vector<double>& vertexSelectValue1,
        std::vector<double>& jointSelectValue1,
        std::vector<double>& vertexSelectValue2,
        std::vector<double>& jointSelectValue2);

template<class Model>
void computeAttachSelectValues(
        const Model& model1,
        const Model& model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent2,
        std::vector<double>& vertexSelectValue1,
        std::vector<double>& jointSelectValue1,
        std::vector<double>& vertexSelectValue2,
        std::vector<double>& jointSelectValue2);

template<class Model>
void computeRemoveSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent,
        const double minThreshold,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue);

template<class Model>
void computeDetachSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent,
        const double minThreshold,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue);


}

#include "skinmixer_select.cpp"

#endif // SKINMIXER_SELECT_H
