#include "skinmixer_blend_skeletons.h"

#include "skinmixer/skinmixer_operation.h"

#include <unordered_set>

#include <nvl/math/interpolation.h>

namespace skinmixer {

template<class Model>
void blendSkeletons(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        typename SkinMixerData<Model>::Entry& entry)
{
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

    typedef typename nvl::Index Index;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Skeleton::Joint Joint;

    typedef typename Skeleton::Transformation Transformation;

    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;

    std::vector<std::vector<JointId>> jointMap(data.entryNumber());
    std::vector<std::unordered_set<JointId>> jointSet(data.entryNumber());
    std::vector<std::unordered_set<JointId>> jointToBeMerged(data.entryNumber());
    std::vector<std::vector<JointId>> joints(data.entryNumber());

    for (const nvl::Index& eId : cluster) {
        const Entry& currentEntry = data.entry(eId);
        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;

        //Get joints to be retrieved using the select values
        for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
            if (currentEntry.select.joint[jId]) {
                joints[eId].push_back(jId);
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

        jointMap[eId] = std::vector<JointId>(currentSkeleton.jointNumber(), nvl::MAX_ID);
        jointSet[eId] = std::unordered_set<JointId>(joints[eId].begin(), joints[eId].end());
    }

    //Recursively add childs
    bool done;
    do {
        done = true;

        for (const nvl::Index& eId : cluster) {
            const Entry& currentEntry = data.entry(eId);
            Model* currentModel = currentEntry.model;
            Skeleton& currentSkeleton = currentModel->skeleton;

            for (JointId i = 0; i < joints[eId].size(); ++i) {
                JointId jId = joints[eId][i];

                if (jointMap[eId][jId] != nvl::MAX_ID)
                    continue;

                done = false;

                assert(jointSet[eId].find(jId) != jointSet[eId].end());

                const Joint& joint = currentSkeleton.joint(jId);
                JointId parent = currentSkeleton.parent(jId);

                JointId newJId = nvl::MAX_ID;
                if (parent == nvl::MAX_ID || (jointSet[eId].find(parent) == jointSet[eId].end() && jointToBeMerged[eId].find(jId) == jointToBeMerged[eId].end())) {
                    newJId = targetSkeleton.addRoot(joint);
                }
                else if (parent != nvl::MAX_ID && jointMap[eId][parent] != nvl::MAX_ID) {
                    const Joint& joint = currentSkeleton.joint(jId);
                    newJId = targetSkeleton.addChild(jointMap[eId][parent], joint);
                }
                if (newJId != nvl::MAX_ID) {
                    jointMap[eId][jId] = newJId;

                    assert(entry.birth.joint.size() == newJId);
                    entry.birth.joint.push_back(std::vector<JointInfo>());

                    JointInfo jInfo;
                    jInfo.eId = eId;
                    jInfo.jId = jId;
                    entry.birth.joint[newJId].push_back(jInfo);

                    std::vector<Transformation> tranformations;
                    tranformations.push_back(joint.restTransform());

                    for (const Index& aId : currentEntry.relatedActions) {
                        const Action& action = data.action(aId);

                        if (action.operation == OperationType::ATTACH) {
                            JointInfo actionJInfo;
                            actionJInfo.eId = nvl::MAX_ID;

                            if (action.entry1 == eId && action.joint1 == jId) {
                                actionJInfo.eId = action.entry2;
                                actionJInfo.jId = action.joint2;
                                jointMap[action.entry2][action.joint2] = newJId;
                            }
                            else if (action.entry2 == eId && action.joint2 == jId) {
                                actionJInfo.eId = action.entry1;
                                actionJInfo.jId = action.joint1;
                                jointMap[action.entry1][action.joint1] = newJId;
                            }
                            if (actionJInfo.eId != nvl::MAX_ID) {
                                entry.birth.joint[newJId].push_back(actionJInfo);
                                tranformations.push_back(data.entry(actionJInfo.eId).model->skeleton.joint(actionJInfo.jId).restTransform());
                            }
                        }
                    }

                    //TODO INTERPOLATE TRANSFORMATIONS
                }
            }
        }
    }
    while (!done);
}

}
