#include "skinmixer_blend_skeletons.h"

#include "skinmixer/skinmixer_operation.h"

#include <unordered_set>

#include <nvl/math/interpolation.h>
#include <nvl/math/normalization.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
void blendSkeletons(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        typename SkinMixerData<Model>::Entry& entry)
{
    const double distanceWeight = 0.02;
    const double directionWeight = 0.05;
    const double topologyWeight = 0.43;
    const double matchedWeight = 1.0 - directionWeight - distanceWeight - topologyWeight;

    typedef typename nvl::Index Index;

    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Skeleton::Joint Joint;

    typedef typename Skeleton::Transformation Transformation;

    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;

    std::vector<std::vector<JointId>> jointMap(data.entryNumber());

    std::vector<std::unordered_set<JointId>> jointToBeMerged(data.entryNumber());

    std::vector<std::set<JointId>> keptJoints(data.entryNumber());
    std::vector<std::set<JointId>> nonKeptJoints(data.entryNumber());

    std::vector<std::unordered_set<JointId>> matchedJoint(data.entryNumber());

    for (const nvl::Index& eId : cluster) {
        const Entry& currentEntry = data.entry(eId);
        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;

        //Get joints to be retrieved using the select values
        for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
            if (currentEntry.select.joint[jId]) {
                keptJoints[eId].insert(jId);
            }
            else {
                nonKeptJoints[eId].insert(jId);
            }
        }

        //Get merge set
        for (const Index& aId : currentEntry.relatedActions) {
            const Action& action = data.action(aId);

            if (action.operation == OperationType::ATTACH) {
                if (action.entry1 == eId) {
                    jointToBeMerged[action.entry1].insert(action.joint1);
                }
                else if (action.entry2 == eId) {
                    jointToBeMerged[action.entry2].insert(action.joint2);
                }
            }
        }

        jointMap[eId] = std::vector<JointId>(currentSkeleton.jointNumber(), nvl::MAX_INDEX);
    }


    std::vector<std::set<JointId>> remainingAssignedJoints = keptJoints;

    bool keptDone;

    do {
        keptDone = true;

        for (const nvl::Index& eId : cluster) {
            const Entry& currentEntry = data.entry(eId);
            const Model* currentModel = currentEntry.model;
            const Skeleton& currentSkeleton = currentModel->skeleton;

            typename std::set<JointId>::iterator it = remainingAssignedJoints[eId].begin();
            while (it != remainingAssignedJoints[eId].end()) {
                JointId jId = *it;

                assert(jointMap[eId][jId] == nvl::MAX_INDEX);
                assert(keptJoints[eId].find(jId) != keptJoints[eId].end() && remainingJoints[eId].find(jId) != remainingJoints[eId].end());

                keptDone = false;

                const Joint& joint = currentSkeleton.joint(jId);
                JointId parentId = currentSkeleton.parentId(jId);

                JointId newJId = nvl::MAX_INDEX;
                if (parentId == nvl::MAX_INDEX || (keptJoints[eId].find(parentId) == keptJoints[eId].end() && jointToBeMerged[eId].find(jId) == jointToBeMerged[eId].end())) {
                    newJId = targetSkeleton.addRoot(joint);
                }
                else if (parentId != nvl::MAX_INDEX && jointMap[eId][parentId] != nvl::MAX_INDEX) {
                    const Joint& joint = currentSkeleton.joint(jId);
                    newJId = targetSkeleton.addChild(jointMap[eId][parentId], joint);
                }
                if (newJId != nvl::MAX_INDEX) {
                    jointMap[eId][jId] = newJId;
                    matchedJoint[eId].insert(newJId);

                    assert(entry.birth.joint.size() == newJId);
                    entry.birth.joint.push_back(std::vector<JointInfo>());

                    JointInfo jInfo;
                    jInfo.eId = eId;
                    jInfo.jId = jId;
                    jInfo.confidence = 1.0;
                    entry.birth.joint[newJId].push_back(jInfo);

                    std::vector<Transformation> transformations;
                    std::vector<double> transformationWeights;
                    transformations.push_back(joint.restTransform());
                    transformationWeights.push_back(1.0);

                    for (const Index& aId : currentEntry.relatedActions) {
                        const Action& action = data.action(aId);

                        if (action.operation == OperationType::ATTACH) {
                            JointInfo actionJInfo;
                            actionJInfo.eId = nvl::MAX_INDEX;
                            actionJInfo.jId = nvl::MAX_INDEX;
                            actionJInfo.confidence = 1.0;

                            if (action.entry1 == eId && action.joint1 == jId) {
                                actionJInfo.eId = action.entry2;
                                actionJInfo.jId = action.joint2;
                            }
                            else if (action.entry2 == eId && action.joint2 == jId) {
                                actionJInfo.eId = action.entry1;
                                actionJInfo.jId = action.joint1;
                            }
                            if (actionJInfo.jId != nvl::MAX_INDEX) {
                                assert(actionJInfo.eId != nvl::MAX_INDEX);

                                jointMap[actionJInfo.eId][actionJInfo.jId] = newJId;
                                entry.birth.joint[newJId].push_back(actionJInfo);

                                transformations.push_back(data.entry(actionJInfo.eId).model->skeleton.joint(actionJInfo.jId).restTransform());
                                transformationWeights.push_back(1.0);

                                remainingAssignedJoints[actionJInfo.eId].erase(actionJInfo.jId);
                                matchedJoint[actionJInfo.eId].insert(newJId);
                            }
                        }
                    }

                    if (transformations.size() > 1) {
                        nvl::normalize(transformationWeights);
                        targetSkeleton.joint(newJId).setRestTransform(nvl::interpolateAffine(transformations, transformationWeights));
                    }

                    remainingAssignedJoints[eId].erase(it++);
                }
                else {
                    ++it;
                }
            }
        }
    }
    while (!keptDone);

    for (const nvl::Index& eId : cluster) {
        const Entry& currentEntry = data.entry(eId);
        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;

        std::set<JointId> remainingJoints = nonKeptJoints[eId];

        double maxDistance = 0.0;
        for (JointId currentJId = 0; currentJId < currentSkeleton.jointNumber(); ++currentJId) {
            const Joint& currentJoint = currentSkeleton.joint(currentJId);

            for (JointId targetJId = 0; targetJId < targetSkeleton.jointNumber(); ++targetJId) {
                const Joint& targetJoint = targetSkeleton.joint(targetJId);
                double distance = ((targetJoint.restTransform() * nvl::Point3d(0,0,0)) - (currentJoint.restTransform() * nvl::Point3d(0,0,0))).norm();
                maxDistance = std::max(maxDistance, distance);
            }
        }

        do {
            double bestScore = nvl::minLimitValue<double>();
            JointId bestCurrentJoint = nvl::MAX_INDEX;
            JointId bestTargetJoint = nvl::MAX_INDEX;

            for (const JointId& currentJId : remainingJoints) {
                assert(jointMap[eId][currentJId] == nvl::MAX_INDEX);
                assert(nonKeptJoints[eId].find(currentJId) != nonKeptJoints[eId].end() && remainingJoints[eId].find(currentJId) != remainingJoints[eId].end());

                const Joint& currentJoint = currentSkeleton.joint(currentJId);

                for (JointId targetJId = 0; targetJId < targetSkeleton.jointNumber(); ++targetJId) {
                    const Joint& targetJoint = targetSkeleton.joint(targetJId);

                    JointId currentParentId = currentSkeleton.parentId(currentJId);
                    JointId targetParentId = targetSkeleton.parentId(targetJId);

                    double distanceScore = 1 - (targetJoint.restTransform() * nvl::Point3d(0,0,0) - currentJoint.restTransform() * nvl::Point3d(0,0,0)).norm() / maxDistance;



                    double parentDirectionScore;
                    if (currentParentId == nvl::MAX_INDEX && targetParentId == nvl::MAX_INDEX) {
                        parentDirectionScore = 1.0;
                    }
                    else if (currentParentId != nvl::MAX_INDEX && targetParentId != nvl::MAX_INDEX) {
                        nvl::Vector3d currentParentDirection = currentSkeleton.joint(currentParentId).restTransform() * nvl::Point3d(0,0,0) - currentJoint.restTransform() * nvl::Point3d(0,0,0);
                        nvl::Vector3d targetParentDirection = targetSkeleton.joint(targetParentId).restTransform() * nvl::Point3d(0,0,0) - targetJoint.restTransform() * nvl::Point3d(0,0,0);
                        parentDirectionScore = currentParentDirection.dot(targetParentDirection);
                    }
                    else {
                        parentDirectionScore = 0.0;
                    }

                    double childrenDirectionScore;
                    if (currentSkeleton.children(currentJId).empty() && targetSkeleton.children(targetJId).empty()) {
                        childrenDirectionScore = 1.0;
                    }
                    else if (!currentSkeleton.children(currentJId).empty() && !targetSkeleton.children(targetJId).empty()) {
                        nvl::Vector3d currentChildrenDirection(0.0, 0.0, 0.0);
                        for (JointId childId : currentSkeleton.children(currentJId)) {
                            currentChildrenDirection += (currentJoint.restTransform() * nvl::Point3d(0,0,0) - currentSkeleton.joint(childId).restTransform() * nvl::Point3d(0,0,0));
                        }
                        currentChildrenDirection /= currentSkeleton.children(currentJId).size();

                        nvl::Vector3d targetChildrenDirection(0.0, 0.0, 0.0);
                        for (JointId childId : targetSkeleton.children(targetJId)) {
                            targetChildrenDirection += (targetJoint.restTransform() * nvl::Point3d(0,0,0) - targetSkeleton.joint(childId).restTransform() * nvl::Point3d(0,0,0));
                        }
                        targetChildrenDirection /= targetSkeleton.children(targetJId).size();

                        childrenDirectionScore = currentChildrenDirection.dot(targetChildrenDirection);
                    }
                    else {
                        childrenDirectionScore = 0.0;
                    }

                    double directionScore = parentDirectionScore * 0.5 + childrenDirectionScore * 0.5;



                    double parentTopologyScore;
                    if (currentParentId == nvl::MAX_INDEX && targetParentId == nvl::MAX_INDEX) {
                        parentTopologyScore = 1.0;
                    }
                    else if (currentParentId != nvl::MAX_INDEX && targetParentId != nvl::MAX_INDEX && jointMap[eId][currentParentId] == targetParentId) {
                        parentTopologyScore = 1.0;
                    }
                    else {
                        parentTopologyScore = 0.0;
                    }

                    double childrenTopologyScore;
                    if (currentSkeleton.children(currentJId).empty() && targetSkeleton.children(targetJId).empty()) {
                        childrenTopologyScore = 1.0;
                    }
                    else if (!currentSkeleton.children(currentJId).empty() && !targetSkeleton.children(targetJId).empty()) {
                        childrenTopologyScore = 0.0;

                        for (JointId currentChildId : currentSkeleton.children(currentJId)) {
                            double childValue = 0.0;
                            for (JointId targetChildId : targetSkeleton.children(targetJId)) {
                                if (jointMap[eId][currentChildId] == targetChildId) {
                                    childValue = 1.0;
                                }
                            }
                            childrenTopologyScore += childValue;
                        }

                        childrenTopologyScore /= currentSkeleton.children(currentJId).size();
                    }
                    else {
                        childrenTopologyScore = 0.0;
                    }

                    double topologyScore = parentTopologyScore * 0.5 + childrenTopologyScore * 0.5;




                    double matchedScore = (matchedJoint[eId].find(targetJId) == matchedJoint[eId].end() ? 1.0 : 0.0);




                    assert(distanceScore >= 0 && distanceScore <= 1);
                    assert(directionScore >= 0 && directionScore <= 1);
                    assert(topologyScore >= 0 && topologyScore <= 1);
                    assert(matchedWeight >= 0 && matchedWeight <= 1);

                    double score = distanceWeight * distanceScore + directionWeight * directionScore + topologyWeight * topologyScore + matchedScore * matchedWeight;
                    assert(score >= 0 && score <= 1);
                    if (score >= bestScore) {
                        bestCurrentJoint = currentJId;
                        bestTargetJoint = targetJId;
                        bestScore = score;
                    }
                }
            }

            assert(bestScore > nvl::minLimitValue<double>());
            jointMap[eId][bestCurrentJoint] = bestTargetJoint;
            matchedJoint[eId].insert(bestTargetJoint);

            assert(!entry.birth.joint[bestTargetJoint].empty());
            JointInfo jInfo;
            jInfo.eId = eId;
            jInfo.jId = bestCurrentJoint;
            jInfo.confidence = bestScore;
            entry.birth.joint[bestTargetJoint].push_back(jInfo);

            remainingJoints.erase(bestCurrentJoint);
        }
        while (!remainingJoints.empty());
    }
}

}
