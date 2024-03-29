#include "skinmixer_blend_skeletons.h"

#include <unordered_set>

#include <nvl/math/interpolation.h>
#include <nvl/math/normalization.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

namespace internal {

template<class Skeleton>
struct SkeletonPaths {
    typedef typename Skeleton::JointId JointId;
    typedef typename Skeleton::Scalar Scalar;

    SkeletonPaths();
    SkeletonPaths(const Skeleton& skeleton);

    void compute(const Skeleton& skeleton);

    std::vector<std::vector<JointId>> paths;
    std::vector<std::unordered_map<JointId, double>> pathDistance;
    std::vector<std::unordered_map<JointId, double>> pathNormalizedDistance;
    std::vector<std::vector<std::pair<nvl::Index, nvl::Index>>> jointToPaths;
};


double skeletonMatchingMatchedScore(
        const bool nonMatched,
        const bool nonPerfectMatched);

template<class Skeleton>
double skeletonMatchingConfidence(
        const Skeleton& currentSkeleton,
        const Skeleton& targetSkeleton,
        const typename Skeleton::JointId& currentJId,
        const typename Skeleton::JointId& targetJId,
        const SkeletonPaths<Skeleton>& targetPaths,
        const SkeletonPaths<Skeleton>& currentPaths,
        const unsigned int& targetTopologicalDistance,
        const unsigned int& maxTopologicalDistance,
        const nvl::Point3<typename Skeleton::Scalar>& currentPivotPoint,
        const nvl::Point3<typename Skeleton::Scalar>& targetPivotPoint);

}

template<class Model>
void blendSkeletons(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry)
{
    typedef typename nvl::Index Index;

    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename SkinMixerData<Model>::SelectInfo SelectInfo;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Skeleton::Joint Joint;

    typedef typename Skeleton::Transformation Transformation;
    typedef typename Skeleton::Scalar Scalar;
    typedef typename nvl::Point3<Scalar> Point;


    Model* targetModel = resultEntry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;

    std::vector<std::vector<JointId>> jointMap(data.entryNumber());

    std::vector<std::unordered_set<JointId>> jointToBeMerged(data.entryNumber());

    std::vector<std::set<JointId>> keptJoints(data.entryNumber());
    std::vector<std::set<JointId>> nonKeptJoints(data.entryNumber());

    std::vector<std::unordered_set<JointId>> matchedJoints(data.entryNumber());
    std::vector<std::unordered_set<JointId>> perfectMatchedJoints(data.entryNumber());
    std::vector<std::unordered_set<JointId>> seedJoints(data.entryNumber());

    for (const Index& birthEId : cluster) {
        const Entry& currentEntry = data.entry(birthEId);

        SelectInfo select = data.computeGlobalSelectInfo(birthEId);

        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;

        //Get joints to be retrieved using the select values
        for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
            if (select.joint[jId] > 0.0 && !nvl::epsEqual(select.joint[jId], 0.0)) {
                keptJoints[birthEId].insert(jId);
            }
            else {
                nonKeptJoints[birthEId].insert(jId);
            }
        }

        //Get merge set
        std::vector<Index> relatedActions = data.relatedActions(currentEntry);
        for (const Index& aId : relatedActions) {
            const Action& action = data.action(aId);

            if (action.entry1 == birthEId) {
                if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                    jointToBeMerged[action.entry1].insert(action.joint1);
                }
                seedJoints[action.entry1].insert(action.joint1);
            }
            else if (action.entry2 == birthEId) {
                if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                    jointToBeMerged[action.entry2].insert(action.joint2);
                }
                seedJoints[action.entry2].insert(action.joint2);
            }
        }

        jointMap[birthEId] = std::vector<JointId>(currentSkeleton.jointNumber(), nvl::NULL_ID);
    }

    std::vector<std::set<JointId>> remainingAssignedJoints = keptJoints;

    bool keptDone;

    do {
        keptDone = true;

        for (const nvl::Index& birthEId : cluster) {
            const Entry& currentEntry = data.entry(birthEId);
            const Model* currentModel = currentEntry.model;
            const Skeleton& currentSkeleton = currentModel->skeleton;

            typename std::set<JointId>::iterator it = remainingAssignedJoints[birthEId].begin();
            while (it != remainingAssignedJoints[birthEId].end()) {
                JointId jId = *it;

                assert(jointMap[birthEId][jId] == nvl::NULL_ID);
                assert(keptJoints[birthEId].find(jId) != keptJoints[birthEId].end() && remainingAssignedJoints[birthEId].find(jId) != remainingAssignedJoints[birthEId].end());

                keptDone = false;

                const Joint& joint = currentSkeleton.joint(jId);
                JointId parentId = currentSkeleton.parentId(jId);

                JointId newJId = nvl::NULL_ID;
                if (parentId == nvl::NULL_ID || (keptJoints[birthEId].find(parentId) == keptJoints[birthEId].end() && jointToBeMerged[birthEId].find(jId) == jointToBeMerged[birthEId].end())) {
                    newJId = targetSkeleton.addRoot(joint);
                }
                else if (parentId != nvl::NULL_ID && jointMap[birthEId][parentId] != nvl::NULL_ID) {
                    const Joint& joint = currentSkeleton.joint(jId);
                    newJId = targetSkeleton.addChild(joint, jointMap[birthEId][parentId]);
                }

                if (newJId != nvl::NULL_ID) {
                    jointMap[birthEId][jId] = newJId;
                    matchedJoints[birthEId].insert(newJId);
                    perfectMatchedJoints[birthEId].insert(newJId);

                    assert(resultEntry.birth.joint.size() == newJId);
                    resultEntry.birth.joint.push_back(std::vector<JointInfo>());

                    JointInfo jInfo;
                    jInfo.eId = birthEId;
                    jInfo.jId = jId;
                    jInfo.confidence = 1.0;
                    resultEntry.birth.joint[newJId].push_back(jInfo);

                    Transformation transformation = joint.bindPose();

                    //Find the and handle merge joints
                    std::vector<Index> relatedActions = data.relatedActions(currentEntry);
                    for (const Index& aId : relatedActions) {
                        const Action& action = data.action(aId);

                        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                            JointInfo actionJInfo;
                            actionJInfo.eId = nvl::NULL_ID;
                            actionJInfo.jId = nvl::NULL_ID;
                            actionJInfo.confidence = 1.0;

                            if (action.entry1 == birthEId && action.joint1 == jId) {
                                actionJInfo.eId = action.entry2;
                                actionJInfo.jId = action.joint2;
                            }
                            else if (action.entry2 == birthEId && action.joint2 == jId) {
                                actionJInfo.eId = action.entry1;
                                actionJInfo.jId = action.joint1;
                            }

                            //Case merge joint
                            if (actionJInfo.jId != nvl::NULL_ID) {
                                jointMap[actionJInfo.eId][actionJInfo.jId] = newJId;

                                resultEntry.birth.joint[newJId].push_back(actionJInfo);

                                transformation = data.entry(action.entry1).model->skeleton.jointBindPose(action.joint1);

                                remainingAssignedJoints[actionJInfo.eId].erase(actionJInfo.jId);
                                matchedJoints[actionJInfo.eId].insert(newJId);
                                perfectMatchedJoints[actionJInfo.eId].insert(newJId);

                                resultEntry.birth.mergeJoints.push_back(newJId);
                            }
                        }
                    }

                    targetSkeleton.joint(newJId).setBindPose(transformation);

                    remainingAssignedJoints[birthEId].erase(it++);
                }
                else {
                    ++it;
                }
            }
        }
    }
    while (!keptDone);


    //Compute paths
    internal::SkeletonPaths<Skeleton> targetPaths(targetSkeleton);
    std::unordered_map<nvl::Index, internal::SkeletonPaths<Skeleton>> birthPaths;
    for (const nvl::Index& birthEId : cluster) {
        const Entry& currentEntry = data.entry(birthEId);
        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;
        internal::SkeletonPaths<Skeleton> paths(currentSkeleton);
        birthPaths.insert(std::make_pair(birthEId, paths));
    }

    for (const nvl::Index& birthEId : cluster) {
        const Entry& currentEntry = data.entry(birthEId);
        Model* currentModel = currentEntry.model;
        Skeleton& currentSkeleton = currentModel->skeleton;
        std::vector<JointId>& currentMap = jointMap[birthEId];
        internal::SkeletonPaths<Skeleton>& currentPaths = birthPaths[birthEId];

        std::set<JointId> remainingJoints = nonKeptJoints[birthEId];

        while (!remainingJoints.empty()) {
            double bestConfidence = nvl::minLimitValue<double>();
            double bestScore = nvl::minLimitValue<double>();
            JointId bestCurrentJoint = nvl::NULL_ID;
            JointId bestTargetJoint = nvl::NULL_ID;

            for (JointId assignedJId = 0; assignedJId < currentSkeleton.jointNumber(); ++assignedJId) {
                if (remainingJoints.find(assignedJId) != remainingJoints.end())
                    continue;

                const JointId& assignedParentId = currentSkeleton.parentId(assignedJId);



                //Handle parents
                if (remainingJoints.find(assignedParentId) != remainingJoints.end()) {
                    assert(currentMap[assignedParentId] == nvl::NULL_ID);
                    assert(nonKeptJoints[birthEId].find(assignedParentId) != nonKeptJoints[birthEId].end() && remainingJoints.find(assignedParentId) != remainingJoints.end());

                    JointId currentJId = assignedParentId;



                    //Find candidates (parents)
                    std::vector<JointId> candidates;
                    std::vector<unsigned int> candidateDistance;

                    JointId candidateJoint = currentMap[assignedJId];

                    unsigned int maxTopologicalDistance = 0;
                    candidates.push_back(candidateJoint);
                    candidateDistance.push_back(maxTopologicalDistance);

                    while (!targetSkeleton.isRoot(candidateJoint)) {
                        candidateJoint = targetSkeleton.parentId(candidateJoint);
                        candidates.push_back(candidateJoint);

                        maxTopologicalDistance++;
                        candidateDistance.push_back(maxTopologicalDistance);
                    }

                    maxTopologicalDistance = std::max(static_cast<unsigned int>(1), maxTopologicalDistance);


                    for (Index candidateId = 0; candidateId < candidates.size(); ++candidateId) { //Compute each candidate
                        const JointId& targetJId = candidates[candidateId];
                        const unsigned int& targetTopologicalDistance = candidateDistance[candidateId];

                        Point currentPivotPoint = currentSkeleton.jointBindPose(assignedJId) * Point::Zero();
                        Point targetPivotPoint = targetSkeleton.jointBindPose(currentMap[assignedJId]) * nvl::Point3d::Zero();

                        double confidence = internal::skeletonMatchingConfidence(
                                    currentSkeleton,
                                    targetSkeleton,
                                    currentJId,
                                    targetJId,
                                    targetPaths,
                                    currentPaths,
                                    targetTopologicalDistance,
                                    maxTopologicalDistance,
                                    currentPivotPoint,
                                    targetPivotPoint);

                        double matchedScore = internal::skeletonMatchingMatchedScore(
                                    matchedJoints[birthEId].find(targetJId) == matchedJoints[birthEId].end(),
                                    perfectMatchedJoints[birthEId].find(targetJId) == perfectMatchedJoints[birthEId].end());

                        double score = confidence + matchedScore;

                        if (score >= bestScore) {
                            bestCurrentJoint = currentJId;
                            bestTargetJoint = targetJId;
                            bestConfidence = confidence;
                            bestScore = score;
                        }
                    }
                }





                //Handle children
                for (JointId currentChildId : currentSkeleton.children(assignedJId)) {

                    if (remainingJoints.find(currentChildId) == remainingJoints.end())
                        continue;

                    assert(currentMap[currentChildId] == nvl::NULL_ID);
                    assert(nonKeptJoints[birthEId].find(currentChildId) != nonKeptJoints[birthEId].end() && remainingJoints.find(currentChildId) != remainingJoints.end());

                    std::vector<JointId> candidates = nvl::skeletonJointDescendants(targetSkeleton, currentMap[assignedJId]);
                    candidates.push_back(currentMap[assignedJId]);
                    std::vector<unsigned int> distanceFromAssignedJoint = nvl::skeletonJointDistance(targetSkeleton, currentMap[assignedJId]);

                    unsigned int maxTopologicalDistance = 1;
                    for (const JointId& candidate : candidates) {
                        maxTopologicalDistance = std::max(distanceFromAssignedJoint[candidate], maxTopologicalDistance);
                    }
                    JointId currentJId = currentChildId;

                    for (Index candidateId = 0; candidateId < candidates.size(); ++candidateId) { //Compute each candidate
                        const JointId& targetJId = candidates[candidateId];
                        const int& targetTopologicalDistance = distanceFromAssignedJoint[targetJId];

                        Point currentPivotPoint = currentSkeleton.jointBindPose(assignedJId) * Point::Zero();
                        Point targetPivotPoint = targetSkeleton.jointBindPose(currentMap[assignedJId]) * nvl::Point3d::Zero();

                        double confidence = internal::skeletonMatchingConfidence(
                                    currentSkeleton,
                                    targetSkeleton,
                                    currentJId,
                                    targetJId,
                                    targetPaths,
                                    currentPaths,
                                    targetTopologicalDistance,
                                    maxTopologicalDistance,
                                    currentPivotPoint,
                                    targetPivotPoint);

                        double matchedScore = internal::skeletonMatchingMatchedScore(
                                    matchedJoints[birthEId].find(targetJId) == matchedJoints[birthEId].end(),
                                    perfectMatchedJoints[birthEId].find(targetJId) == perfectMatchedJoints[birthEId].end());

                        double score = confidence + matchedScore;

                        if (score >= bestScore) {
                            bestCurrentJoint = currentJId;
                            bestTargetJoint = targetJId;
                            bestConfidence = confidence;
                            bestScore = score;
                        }
                    }
                }
            }


            //Case not connected component
            if (bestConfidence == nvl::minLimitValue<double>()) {

                for (JointId currentJId = 0; currentJId < currentSkeleton.jointNumber(); ++currentJId) {
                    if (remainingJoints.find(currentJId) != remainingJoints.end()) {
                        for (JointId targetJId = 0; targetJId < targetSkeleton.jointNumber(); ++targetJId) {

                            double confidence = internal::skeletonMatchingConfidence(
                                        currentSkeleton,
                                        targetSkeleton,
                                        currentJId,
                                        targetJId,
                                        targetPaths,
                                        currentPaths,
                                        nvl::maxLimitValue<unsigned int>(),
                                        nvl::maxLimitValue<unsigned int>(),
                                        Point::Zero(),
                                        Point::Zero());

                            double matchedScore = internal::skeletonMatchingMatchedScore(
                                        matchedJoints[birthEId].find(targetJId) == matchedJoints[birthEId].end(),
                                        perfectMatchedJoints[birthEId].find(targetJId) == perfectMatchedJoints[birthEId].end());

                            double score = confidence + matchedScore;

                            if (score >= bestScore) {
                                bestCurrentJoint = currentJId;
                                bestTargetJoint = targetJId;
                                bestConfidence = confidence;
                                bestScore = score;
                            }
                        }
                    }
                }
            }

            assert(bestConfidence > nvl::minLimitValue<double>());
            currentMap[bestCurrentJoint] = bestTargetJoint;
            matchedJoints[birthEId].insert(bestTargetJoint);

            assert(!resultEntry.birth.joint[bestTargetJoint].empty());
            JointInfo jInfo;
            jInfo.eId = birthEId;
            jInfo.jId = bestCurrentJoint;
            jInfo.confidence = std::min(0.9999, bestConfidence);
            resultEntry.birth.joint[bestTargetJoint].push_back(jInfo);

            remainingJoints.erase(bestCurrentJoint);
        }
        for (JointId jId = 0; jId < currentSkeleton.jointNumber(); ++jId) {
            assert(currentMap[jId] != nvl::NULL_ID);
        }
    }
}

namespace internal {

template<class Skeleton>
SkeletonPaths<Skeleton>::SkeletonPaths()
{

}

template<class Skeleton>
SkeletonPaths<Skeleton>::SkeletonPaths(const Skeleton& skeleton)
{
    compute(skeleton);
}

template<class Skeleton>
void SkeletonPaths<Skeleton>::compute(const Skeleton& skeleton)
{
    paths = nvl::skeletonRootLeafPaths(skeleton);

    jointToPaths.clear();
    jointToPaths.resize(skeleton.jointNumber());

    pathDistance.clear();
    pathDistance.resize(paths.size());

    pathNormalizedDistance.clear();
    pathNormalizedDistance.resize(paths.size());

    for (nvl::Index i = 0; i < paths.size(); ++i) {
        for (nvl::Index j = 0; j < paths[i].size(); ++j) {
            const JointId& jId = paths[i][j];
            jointToPaths[jId].push_back(std::make_pair(i,j));
        }

        for (nvl::Index j = 0; j < paths[i].size(); ++j) {
            const JointId& jId = paths[i][j];

            double distance = 0.0;
            if (j > 0) {
                assert(j > 0);
                assert(!skeleton.isRoot(jId));
                const JointId& parentId = skeleton.parentId(jId);

                nvl::Vector3<Scalar> p = nvl::Vector3<Scalar>::Zero();
                p = skeleton.jointBindPose(parentId).inverse() * skeleton.jointBindPose(jId) * p;

                distance = pathDistance[i].at(parentId) + p.norm();
            }
            pathDistance[i].insert(std::make_pair(jId, distance));
        }

        double maxDistance = pathDistance[i].at(paths[i][paths[i].size() - 1]);
        for (nvl::Index j = 0; j < paths[i].size(); ++j) {
            const JointId& jId = paths[i][j];
            double distance =  pathDistance[i].at(jId) / maxDistance;
            pathNormalizedDistance[i].insert(std::make_pair(jId, distance));
        }
    }
}

inline double skeletonMatchingMatchedScore(
        const bool nonMatched,
        const bool nonPerfectMatched)
{
    double alreadyMatchedScore = (nonMatched ? 1.0 : 0.0);
    double perfectMatchedScore = (nonPerfectMatched ? 1.0 : 0.0);

    double matchedScore =
            0.5 * perfectMatchedScore +
            0.1 * alreadyMatchedScore;

    return matchedScore;
}

template<class Skeleton>
double skeletonMatchingConfidence(
        const Skeleton& currentSkeleton,
        const Skeleton& targetSkeleton,
        const typename Skeleton::JointId& currentJId,
        const typename Skeleton::JointId& targetJId,
        const SkeletonPaths<Skeleton>& targetPaths,
        const SkeletonPaths<Skeleton>& currentPaths,
        const unsigned int& targetTopologicalDistance,
        const unsigned int& maxTopologicalDistance,
        const nvl::Point3<typename Skeleton::Scalar>& currentPivotPoint,
        const nvl::Point3<typename Skeleton::Scalar>& targetPivotPoint)
{
    typedef typename Skeleton::JointId JointId;
    typedef typename nvl::Index Index;
    typedef typename Skeleton::Scalar Scalar;
    typedef typename nvl::Point3<Scalar> Point;

    //Path score
    double pathScore = 0.0;
    //Both roots
    if (currentSkeleton.isRoot(currentJId) && targetSkeleton.isRoot(targetJId)) {
        pathScore = 1.0;
    }
    else {
        const std::vector<std::pair<Index, Index>>& targetPassingPaths = targetPaths.jointToPaths[targetJId];
        for (const std::pair<Index, Index>& targetPassingPath : targetPassingPaths) {
            JointId targetLastJointInPath = targetPaths.paths[targetPassingPath.first][targetPaths.paths[targetPassingPath.first].size() - 1];
            double targetMaxDistance = targetPaths.pathDistance[targetPassingPath.first].at(targetLastJointInPath);
            double targetPathDistance = targetPaths.pathDistance[targetPassingPath.first].at(targetJId);
            double targetNormalizedDistance = targetPaths.pathNormalizedDistance[targetPassingPath.first].at(targetJId);


            double bestPathScore = nvl::minLimitValue<double>();
            const std::vector<std::pair<Index, Index>>& currentPassingPaths = currentPaths.jointToPaths[currentJId];

            for (const std::pair<Index, Index>& currentPassingPath : currentPassingPaths) {
                JointId currentLastJointInPath = currentPaths.paths[currentPassingPath.first][currentPaths.paths[currentPassingPath.first].size() - 1];
                double currentMaxDistance = currentPaths.pathDistance[currentPassingPath.first].at(currentLastJointInPath);
                double currentPathDistance = currentPaths.pathDistance[currentPassingPath.first].at(currentJId);
                double currentNormalizedDistance = currentPaths.pathNormalizedDistance[currentPassingPath.first].at(currentJId);

                double currentPathScore = 1.0 - (0.05 * std::fabs(currentPathDistance - targetPathDistance) / std::max(currentMaxDistance, targetMaxDistance) + 0.95 * std::fabs(currentNormalizedDistance - targetNormalizedDistance));

                if (currentPathScore >= bestPathScore) {
                    bestPathScore = currentPathScore;
                }
            }

            if (bestPathScore > nvl::minLimitValue<double>()) {
                pathScore += bestPathScore;
            }
        }

        pathScore /= static_cast<double>(targetPassingPaths.size());
    }
    assert(pathScore >= 0 - nvl::EPSILON && pathScore <= 1 + nvl::EPSILON);



    double topologyScore;
    //Both roots
    if (currentSkeleton.isRoot(currentJId) && targetSkeleton.isRoot(targetJId)) {
        topologyScore = 1.0;
    }
    //No roots
    else if (!currentSkeleton.isRoot(currentJId) && !targetSkeleton.isRoot(targetJId)) {
        if (targetTopologicalDistance < nvl::maxLimitValue<unsigned int>()) {
            int distanceValue = nvl::abs(static_cast<int>(targetTopologicalDistance) - 1);
            assert(static_cast<int>(maxTopologicalDistance) > 0 && distanceValue <= static_cast<int>(maxTopologicalDistance));
            topologyScore = 1.0 - (static_cast<double>(distanceValue) / static_cast<double>(maxTopologicalDistance));
        }
        else if (currentSkeleton.isLeaf(currentJId) && targetSkeleton.isLeaf(targetJId)) {
            topologyScore = 1.0;
        }
        else {
            topologyScore = 0.0;
        }
    }
    //One is root, the other is not
    else {
        topologyScore = 0.0;
    }
    assert(topologyScore >= 0 - nvl::EPSILON && topologyScore <= 1 + nvl::EPSILON);



    double childrenNumberScore;
    //Both no children
    if (currentSkeleton.children(currentJId).empty() && targetSkeleton.children(targetJId).empty()) {
        childrenNumberScore = 1.0;
    }
    //Otherwise
    else {
        childrenNumberScore =
            static_cast<double>(std::min(currentSkeleton.children(currentJId).size(), targetSkeleton.children(targetJId).size())) /
            static_cast<double>(std::max(currentSkeleton.children(currentJId).size(), targetSkeleton.children(targetJId).size()));
    }
    assert(childrenNumberScore >= 0 - nvl::EPSILON && childrenNumberScore <= 1 + nvl::EPSILON);



    double relativeDirectionScore = 0.0;
    if (currentSkeleton.isRoot(currentJId) && targetSkeleton.isRoot(targetJId)) {
        relativeDirectionScore = 1.0;
    }
    else if (!currentSkeleton.isRoot(currentJId) && !targetSkeleton.isRoot(targetJId)) {
        nvl::Vector3d currentParentDirection = currentSkeleton.jointBindPose(currentJId) * Point::Zero() - currentPivotPoint;
        currentParentDirection.normalize();
        nvl::Vector3d targetParentDirection = targetSkeleton.jointBindPose(targetJId) * nvl::Point3d::Zero() - targetPivotPoint;
        targetParentDirection.normalize();
        relativeDirectionScore = (currentParentDirection.dot(targetParentDirection) + 1) / 2.0;
    }
    else {
        relativeDirectionScore = 0.0;
    }
    assert(relativeDirectionScore >= 0 - nvl::EPSILON && relativeDirectionScore <= 1 + nvl::EPSILON);



    double globalDirectionScore = 0.0;
    if (currentSkeleton.isRoot(currentJId) && targetSkeleton.isRoot(targetJId)) {
        globalDirectionScore = 1.0;
    }
    else if (!currentSkeleton.isRoot(currentJId) && !targetSkeleton.isRoot(targetJId)) {
        JointId targetRootId = targetJId;
        while (!targetSkeleton.isRoot(targetRootId)) {
            targetRootId = targetSkeleton.parentId(targetRootId);
        }
        JointId currentRootId = currentJId;
        while (!currentSkeleton.isRoot(currentRootId)) {
            currentRootId = currentSkeleton.parentId(currentRootId);
        }

        nvl::Vector3d currentParentDirection = currentSkeleton.jointBindPose(currentJId) * nvl::Point3d::Zero() - currentSkeleton.jointBindPose(currentRootId) * nvl::Point3d::Zero();
        currentParentDirection.normalize();
        nvl::Vector3d targetParentDirection = targetSkeleton.jointBindPose(targetJId) * nvl::Point3d::Zero() - targetSkeleton.jointBindPose(targetRootId) * nvl::Point3d::Zero();
        targetParentDirection.normalize();
        globalDirectionScore = (currentParentDirection.dot(targetParentDirection) + 1) / 2.0;
    }
    else {
        globalDirectionScore = 0.0;
    }
    assert(globalDirectionScore >= 0 - nvl::EPSILON && globalDirectionScore <= 1 + nvl::EPSILON);



    double confidence =
            0.35 * pathScore +
            0.3 * topologyScore +
            0.15 * relativeDirectionScore +
            0.1 * globalDirectionScore +
            0.1 * childrenNumberScore;

    assert(confidence >= 0 - nvl::EPSILON && confidence <= 1 + nvl::EPSILON);

    return confidence;
}

}


}
