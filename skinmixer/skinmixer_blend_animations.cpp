#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_map.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>
#include <nvl/math/numeric_limits.h>

#include <nvl/models/animation_algorithms.h>
#include <iostream>

#define KEYFRAME_SELECTION_VERBOSITY
#define SIMILARITY_WITH_INTERPOLATED_TRANSFORMATION
#define DUPLICATE_KEYFRAME_TO_BLEND nvl::MAX_INDEX - 1

namespace skinmixer {

namespace internal {

template<class Frame, class JointId>
typename Frame::Transformation computeMappedTransformation(
        const Frame& frame,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence);

template<class T>
double transformationSimilarityScore(
        const T& fixedInterpolatedTransformation,
        const T& candidateInterpolatedTransformation,
        const double& fixedTime1,
        const T& fixedTransformation1,
        const double& fixedTime2,
        const T& fixedTransformation2,
        const double& candidateTime1,
        const T& candidateTransformation1,
        const double& candidateTime2,
        const T& candidateTransformation2);

double computeDistanceWeight(const double distance);

}

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& newEntries)
{
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    for (const Index& eId : newEntries) {
        Entry& entry = data.entry(eId);

        Model* targetModel = entry.model;
        Skeleton& targetSkeleton = targetModel->skeleton;

        std::vector<Index>& birthEntries = entry.birth.entries;
        std::vector<Index>& animationsIds = entry.blendingAnimationIds;
        std::vector<Index>& animationsModes = entry.blendingAnimationModes;
        std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

        animationWeights.resize(targetSkeleton.jointNumber(), std::vector<double>(birthEntries.size(), 0.0));

        std::vector<Index> clusterMap = nvl::inverseMap(birthEntries);
        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            JointId parentId = targetSkeleton.parentId(jId);

            int numConfidence = 0;
            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

                const Index& cId = clusterMap[jointInfo.eId];

                if (jointInfo.confidence == 1.0) {
                    animationWeights[jId][cId] = 1.0;
                    numConfidence++;
                }
            }
            if (numConfidence > 1) {
                for (JointInfo jointInfo : jointInfos) {
                    assert(jointInfo.jId != nvl::MAX_INDEX);
                    assert(jointInfo.eId != nvl::MAX_INDEX);
                    assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);
                    assert(parentId != nvl::MAX_INDEX);

                    const Index& cId = clusterMap[jointInfo.eId];

                    const std::vector<JointInfo>& parentJointInfos = entry.birth.joint[parentId];

                    for (JointInfo parentJointInfo : parentJointInfos) {
                        const Index& parentCId = clusterMap[parentJointInfo.eId];

                        if (jointInfo.confidence == 1.0 && parentJointInfo.confidence == 1.0 && parentCId == cId) {
                            animationWeights[jId][cId] = 0.0;
                        }
                    }
                }
            }

            nvl::normalize(animationWeights[jId]);
        }

        animationsModes.resize(birthEntries.size(), BLEND_ANIMATION_FIXED);
        animationsIds.resize(birthEntries.size(), BLEND_ANIMATION_NONE);
    }
}

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        std::vector<std::pair<nvl::Index, nvl::Index>>& resultAnimations)
{    
    const double fps = 60;

    typedef typename nvl::Index Index;

    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    typedef typename Model::Animation Animation;
    typedef typename Animation::Frame Frame;
    typedef typename Animation::Transformation Transformation;

    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;
    std::vector<Index>& cluster = entry.birth.entries;
    const std::vector<Index>& animationIds = entry.blendingAnimationIds;
    const std::vector<Index>& animationModes = entry.blendingAnimationModes;
    const std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

    //Cluster to entry map
    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

    //Resulting animations
    Animation targetAnimation;
    std::vector<Animation> clusterAnimations(cluster.size());

    //Data for fixed and candidate frames for each cluster
    std::vector<std::vector<Frame>> fixedFrames(cluster.size());
    std::vector<std::vector<std::vector<Frame>>> candidateFrames(cluster.size());

    //Fill candidate and fixed frames
    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        Index animationMode = animationModes[cId];
        Index animationId = animationIds[cId];

        //Fixed mode
        if (animationMode == BLEND_ANIMATION_FIXED) {
            if (animationId != BLEND_ANIMATION_NONE) {
                const Animation& currentAnimation = currentModel->animation(animationId);
                for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                    Frame frame = currentAnimation.keyframe(fId);

                    fixedFrames[cId].push_back(frame);
                }
            }
        }
        //Best keyframe or best sliding mode
        else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
            candidateFrames[cId].resize(currentModel->animationNumber());
            for (Index aId = 0; aId < currentModel->animationNumber(); ++aId) {
                if (animationId == BLEND_ANIMATION_NONE|| animationId == aId) {
                    assert(animationId == BLEND_ANIMATION_NONE || (animationId >= 0 && animationId < currentModel->animationNumber()));
                    const Animation& currentAnimation = currentModel->animation(aId);
                    for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                        Frame frame = currentAnimation.keyframe(fId);
                        candidateFrames[cId][aId].push_back(frame);
                    }
                }
            }
        }

        //Blend frames to a given number of fps
        nvl::animationBlendFrameTransformations(fixedFrames[cId], fps);
        for (Index aId = 0; aId < candidateFrames[cId].size(); aId++) {
            nvl::animationBlendFrameTransformations(candidateFrames[cId][aId], fps);
        }

        //Add times of fixed frames
        for (Index fId = 0; fId < fixedFrames[cId].size(); ++fId) {
            times.push_back(fixedFrames[cId][fId].time());
        }
    }

    //At least one time entry
    if (times.empty())
        times.push_back(0.0);

    //Sort and remove duplicates
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());


    //Compute global frames
    std::vector<std::vector<Frame>> globalFixedFrames = fixedFrames;
    std::vector<std::vector<std::vector<Frame>>> globalCandidateFrames = candidateFrames;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Model* currentModel = data.entry(cluster[cId]).model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        nvl::animationComputeGlobalFrames(currentSkeleton, globalFixedFrames[cId]);

        for (Index aId = 0; aId < candidateFrames[cId].size(); aId++) {
            nvl::animationComputeGlobalFrames(currentSkeleton, globalCandidateFrames[cId][aId]);
        }
    }

    //Data for determining the best match
    std::vector<std::vector<double>> bestKeyframeScore(times.size());
    std::vector<std::vector<Index>> bestKeyframeAnimation(times.size());
    std::vector<std::vector<Index>> bestKeyframe(times.size());
    std::vector<std::vector<double>> bestKeyframeAlpha(times.size());

    //Variables for iteration
    std::vector<std::vector<Index>> currentFrameId(times.size());
    std::vector<std::vector<double>> currentTimeOffset(times.size());

    //For each time entry find the best keyframes
    for (Index i = 0; i < times.size(); ++i) {
        //Resize data for determining the best match
        bestKeyframeScore[i].resize(cluster.size(), nvl::minLimitValue<double>());
        bestKeyframeAnimation[i].resize(cluster.size(), nvl::MAX_INDEX);
        bestKeyframe[i].resize(cluster.size(), nvl::MAX_INDEX);
        bestKeyframeAlpha[i].resize(cluster.size(), nvl::minLimitValue<double>());

        //Iteration variables
        currentFrameId[i].resize(cluster.size(), 0);
        currentTimeOffset[i].resize(cluster.size(), 0.0);
    }

    //For each time entry find the current frame for the fixed ones
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];

        //Update corresponding frame for the current time
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const std::vector<Frame>& currentFixedFrames = fixedFrames[cId];
            while (currentFrameId[i][cId] < currentFixedFrames.size() && currentFixedFrames[currentFrameId[i][cId]].time() + currentTimeOffset[i][cId] <= currentTime) {
                ++currentFrameId[i][cId];

                if (currentFrameId[i][cId] >= currentFixedFrames.size()) {
                    currentTimeOffset[i][cId] += currentFixedFrames[currentFixedFrames.size() - 1].time();
                    currentFrameId[i][cId] = 0;
                }
            }
        }
    }

    //Joint distance from the merge joints
    std::vector<unsigned int> targetJointDistance = nvl::skeletonJointDistance(targetSkeleton, entry.birth.mergeJoints);




    // ------------------------------------------ BEST LOOP ------------------------------------------


    //Compute best frames for select best keyframe animation
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index animationMode = animationModes[cId];
        Index animationId = animationIds[cId];

        if (animationMode == BLEND_ANIMATION_LOOP) {
            const Model* currentModel = data.entry(cluster[cId]).model;

            std::vector<Index> candidateAnimations;
            if (animationId == BLEND_ANIMATION_NONE) {
                for (Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                    candidateAnimations.push_back(candidateAId);
                }
            }
            else {
                candidateAnimations.push_back(animationId);
            }

            double bestLoopScore = nvl::minLimitValue<double>();
            Index bestLoopAnimationId = nvl::MAX_INDEX;
            Index bestLoopStartingFrame = nvl::MAX_INDEX;
            std::vector<double> bestLoopScoreSingle;

            for (const Index& candidateAId : candidateAnimations) {
                const std::vector<Frame>& currentCandidateFrames = globalCandidateFrames[cId][candidateAId];

                for (Index startingFId = 0; startingFId < currentCandidateFrames.size(); startingFId++) {
                    const double startingTime = currentCandidateFrames[startingFId].time();

                    Index loopFrameId = startingFId;
                    double loopFrameOffset = -startingTime;

                    double loopScore = 0.0;
                    std::vector<double> loopScoreSingle(times.size(), 0.0);


                    for (Index i = 0; i < times.size(); ++i) {
                        const double& currentTime = times[i];

                        while (loopFrameId < currentCandidateFrames.size() && currentCandidateFrames[loopFrameId].time() + loopFrameOffset <= currentTime) {
                            ++loopFrameId;

                            if (loopFrameId >= currentCandidateFrames.size()) {
                                loopFrameOffset += currentCandidateFrames[currentCandidateFrames.size() - 1].time();
                                loopFrameId = 0;
                            }
                        }

                        const Frame& candidateFrame1 = currentCandidateFrames[loopFrameId == 0 ? currentCandidateFrames.size() - 1 : loopFrameId - 1];
                        const Frame& candidateFrame2 = currentCandidateFrames[loopFrameId];

                        double candidateTime1 = candidateFrame1.time() + loopFrameOffset;
                        double candidateTime2 = candidateFrame2.time() + loopFrameOffset;
                        if (loopFrameId == 0) {
                            candidateTime1 = candidateTime2;
                        }

                        double candidateAlpha = candidateTime1 == candidateTime2 ? 1.0 : (currentTime - candidateTime1) / (candidateTime2 - candidateTime1);
                        assert(candidateAlpha >= 0.0 && candidateAlpha <= 1.0);




                        //For each joint
                        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                            unsigned int numOtherAnimations = 0;
                            double jointFrameScore = 0.0;

                            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                            //Compute distance weights
                            double distanceWeight = internal::computeDistanceWeight(targetJointDistance[jId]);
                            if (distanceWeight == 0.0) {
                                continue;
                            }

                            //Calculate mapped joints and their weights for each entry
                            std::vector<std::vector<JointId>> mappedJoints(cluster.size());
                            std::vector<std::vector<double>> mappedJointConfidence(cluster.size());
                            for (JointInfo jointInfo : jointInfos) {
                                mappedJoints[clusterMap[jointInfo.eId]].push_back(jointInfo.jId);
                                mappedJointConfidence[clusterMap[jointInfo.eId]].push_back(jointInfo.confidence);
                            }

        #ifdef SIMILARITY_WITH_INTERPOLATED_TRANSFORMATION
                            Transformation candidateTransformation1 = internal::computeMappedTransformation(candidateFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation candidateTransformation2 = internal::computeMappedTransformation(candidateFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            Transformation candidateInterpolatedTransformation = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);

                            for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                Index otherAnimationMode = animationModes[otherCId];
                                Index otherAnimationId = animationIds[otherCId];
                                if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_NONE) {
                                    const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                    const Frame& fixedFrame1 = currentFixedFrames[currentFrameId[i][otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[i][otherCId] - 1];
                                    const Frame& fixedFrame2 = currentFixedFrames[currentFrameId[i][otherCId]];

                                    double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][otherCId];
                                    double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][otherCId];
                                    if (currentFrameId[i][otherCId] == 0) {
                                        fixedTime1 = fixedTime2;
                                    }

                                    Transformation fixedTransformation1 = internal::computeMappedTransformation(fixedFrame1, mappedJoints[otherCId], mappedJointConfidence[otherCId]);
                                    Transformation fixedTransformation2 = internal::computeMappedTransformation(fixedFrame2, mappedJoints[otherCId], mappedJointConfidence[otherCId]);

                                    double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                                    assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                                    Transformation fixedTnterpolatedTransformation = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);

                                    double similarity = internal::transformationSimilarityScore(fixedTnterpolatedTransformation, candidateInterpolatedTransformation, fixedTime1, fixedTransformation1, fixedTime2, fixedTransformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                    assert(similarity >= 0 && similarity <= 1);

                                    jointFrameScore += similarity;

                                    numOtherAnimations++;
                                }
                            }
        #else
                            std::vector<std::vector<double>> normalizedMappedJointConfidence = mappedJointConfidence;
                            for (Index mi = 0; mi < mappedJoints.size(); ++mi) {
                                nvl::normalize(normalizedMappedJointConfidence[mi]);
                            }

                            for (Index mi = 0; mi < mappedJoints[cId].size(); ++mi) {
                                const JointId& candidateMappedJoint = mappedJoints[cId][mi];
                                const JointId& candidateMappedJointConfidence = normalizedMappedJointConfidence[cId][mi];

                                const Transformation& candidateTransformation1 = candidateFrame1.transformation(candidateMappedJoint);
                                const Transformation& candidateTransformation2 = candidateFrame2.transformation(candidateMappedJoint);

                                Transformation candidateInterpolatedTransformation = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);

                                for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                    Index otherAnimationMode = animationModes[otherCId];
                                    Index otherAnimationId = animationIds[otherCId];
                                    if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_NONE) {
                                        for (Index mj = 0; mj < mappedJoints[otherCId].size(); ++mj) {
                                            const JointId& currentMappedJoint = mappedJoints[otherCId][mj];
                                            const JointId& currentMappedJointConfidence = normalizedMappedJointConfidence[otherCId][mj];

                                            const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                            const Frame& fixedFrame1 = currentFixedFrames[currentFrameId[i][otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[i][otherCId] - 1];
                                            const Frame& fixedFrame2 = currentFixedFrames[currentFrameId[i][otherCId]];

                                            double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][otherCId];
                                            double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][otherCId];
                                            if (currentFrameId[i][otherCId] == 0) {
                                                fixedTime1 = fixedTime2;
                                            }

                                            const Transformation& fixedTransformation1 = fixedFrame1.transformation(currentMappedJoint);
                                            const Transformation& fixedTransformation2 = fixedFrame2.transformation(currentMappedJoint);

                                            double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                                            assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                                            Transformation fixedTnterpolatedTransformation = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);

                                            double similarity = internal::transformationSimilarityScore(fixedTnterpolatedTransformation, candidateInterpolatedTransformation, fixedTime1, fixedTransformation1, fixedTime2, fixedTransformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                            assert(similarity >= 0 && similarity <= 1);

                                            jointFrameScore += currentMappedJointConfidence * candidateMappedJointConfidence * similarity;
                                        }

                                        numOtherAnimations++;
                                    }
                                }
                            }
        #endif
                            if (numOtherAnimations > 0) {
                                jointFrameScore /= static_cast<double>(numOtherAnimations);
                            }

                            assert(jointFrameScore >= 0 && jointFrameScore <= 1);
                            loopScore += distanceWeight * jointFrameScore;
                            loopScoreSingle[i] = distanceWeight * jointFrameScore;
                        }
                    }

                    if (loopScore > bestLoopScore) {
                        bestLoopScore = loopScore;
                        bestLoopAnimationId = candidateAId;
                        bestLoopStartingFrame = startingFId;
                        bestLoopScoreSingle = loopScoreSingle;
                    }
                }
            }

            if (bestLoopAnimationId != nvl::MAX_INDEX) {
                const std::vector<Frame>& currentCandidateFrames = globalCandidateFrames[cId][bestLoopAnimationId];

                const double startingTime = currentCandidateFrames[bestLoopStartingFrame].time();

                Index loopFrameId = bestLoopStartingFrame;
                double loopFrameOffset = -startingTime;
                for (Index i = 0; i < times.size(); ++i) {
                    const double& currentTime = times[i];

                    while (loopFrameId < currentCandidateFrames.size() && currentCandidateFrames[loopFrameId].time() + loopFrameOffset <= currentTime) {
                        ++loopFrameId;

                        if (loopFrameId >= currentCandidateFrames.size()) {
                            loopFrameOffset += currentCandidateFrames[currentCandidateFrames.size() - 1].time();
                            loopFrameId = 0;
                        }
                    }

                    const Frame& candidateFrame1 = currentCandidateFrames[loopFrameId == 0 ? currentCandidateFrames.size() - 1 : loopFrameId - 1];
                    const Frame& candidateFrame2 = currentCandidateFrames[loopFrameId];

                    double candidateTime1 = candidateFrame1.time() + loopFrameOffset;
                    double candidateTime2 = candidateFrame2.time() + loopFrameOffset;
                    if (loopFrameId == 0) {
                        candidateTime1 = candidateTime2;
                    }

                    double candidateAlpha = candidateTime1 == candidateTime2 ? 1.0 : (currentTime - candidateTime1) / (candidateTime2 - candidateTime1);
                    assert(candidateAlpha >= 0.0 && candidateAlpha <= 1.0);

                    bestKeyframeScore[i][cId] = bestLoopScoreSingle[i];
                    bestKeyframeAnimation[i][cId] = bestLoopAnimationId;
                    bestKeyframe[i][cId] = loopFrameId;
                    bestKeyframeAlpha[i][cId] = candidateAlpha;
                }
            }
        }
    }




    // ------------------------------------------ BEST KEYFRAMES ------------------------------------------

    //For each time entry find the best keyframes
    for (Index i = 0; i < times.size(); ++i) {        
        const double& currentTime = times[i];

        for (Index cId = 0; cId < cluster.size(); ++cId) {
            Index animationMode = animationModes[cId];
            Index animationId = animationIds[cId];

            const Model* currentModel = data.entry(cluster[cId]).model;

            if (animationMode == BLEND_ANIMATION_KEYFRAME) {
                std::vector<Index> candidateAnimations;
                if (animationId == BLEND_ANIMATION_NONE) {
                    for (Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                        candidateAnimations.push_back(candidateAId);
                    }
                }
                else {
                    candidateAnimations.push_back(animationId);
                }

                for (const Index& candidateAId : candidateAnimations) {
                    const std::vector<Frame>& currentCandidateFrames = globalCandidateFrames[cId][candidateAId];
                    for (Index candidateFId = 0; candidateFId < currentCandidateFrames.size(); candidateFId++) {
                        const Frame& candidateFrame1 = currentCandidateFrames[candidateFId == 0 ? currentCandidateFrames.size() - 1 : candidateFId - 1];
                        const Frame& candidateFrame2 = currentCandidateFrames[candidateFId];
                        double candidateTime1 = candidateFrame1.time();
                        double candidateTime2 = candidateFrame2.time();

                        double frameScore = 0.0;

                        //For each joint
                        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                            unsigned int numOtherAnimations = 0;
                            double jointFrameScore = 0.0;

                            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                            //Compute distance weights
                            double distanceWeight = internal::computeDistanceWeight(targetJointDistance[jId]);
                            if (distanceWeight == 0.0) {
                                continue;
                            }

                            //Calculate mapped joints and their weights for each entry
                            std::vector<std::vector<JointId>> mappedJoints(cluster.size());
                            std::vector<std::vector<double>> mappedJointConfidence(cluster.size());
                            for (JointInfo jointInfo : jointInfos) {
                                mappedJoints[clusterMap[jointInfo.eId]].push_back(jointInfo.jId);
                                mappedJointConfidence[clusterMap[jointInfo.eId]].push_back(jointInfo.confidence);
                            }

#ifdef SIMILARITY_WITH_INTERPOLATED_TRANSFORMATION
                            Transformation candidateTransformation1 = internal::computeMappedTransformation(candidateFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation candidateTransformation2 = internal::computeMappedTransformation(candidateFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                Index otherAnimationMode = animationModes[otherCId];
                                Index otherAnimationId = animationIds[otherCId];
                                if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_NONE) {
                                    const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                    const Frame& fixedFrame1 = currentFixedFrames[currentFrameId[i][otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[i][otherCId] - 1];
                                    const Frame& fixedFrame2 = currentFixedFrames[currentFrameId[i][otherCId]];

                                    double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][otherCId];
                                    double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][otherCId];
                                    if (currentFrameId[i][otherCId] == 0) {
                                        fixedTime1 = fixedTime2;
                                    }

                                    Transformation fixedTransformation1 = internal::computeMappedTransformation(fixedFrame1, mappedJoints[otherCId], mappedJointConfidence[otherCId]);
                                    Transformation fixedTransformation2 = internal::computeMappedTransformation(fixedFrame2, mappedJoints[otherCId], mappedJointConfidence[otherCId]);

                                    double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                                    assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                                    Transformation fixedTnterpolatedTransformation = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);

                                    double similarity = internal::transformationSimilarityScore(fixedTnterpolatedTransformation, candidateTransformation2, fixedTime1, fixedTransformation1, fixedTime2, fixedTransformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                    assert(similarity >= 0 && similarity <= 1);

                                    jointFrameScore += similarity;

                                    numOtherAnimations++;
                                }
                            }
#else
                            std::vector<std::vector<double>> normalizedMappedJointConfidence = mappedJointConfidence;
                            for (Index mi = 0; mi < mappedJoints.size(); ++mi) {
                                nvl::normalize(normalizedMappedJointConfidence[mi]);
                            }

                            for (Index mi = 0; mi < mappedJoints[cId].size(); ++mi) {
                                const JointId& candidateMappedJoint = mappedJoints[cId][mi];
                                const JointId& candidateMappedJointConfidence = normalizedMappedJointConfidence[cId][mi];

                                const Transformation& candidateTransformation1 = candidateFrame1.transformation(candidateMappedJoint);
                                const Transformation& candidateTransformation2 = candidateFrame2.transformation(candidateMappedJoint);

                                for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                    Index otherAnimationMode = animationModes[otherCId];
                                    Index otherAnimationId = animationIds[otherCId];
                                    if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_NONE) {
                                        for (Index mj = 0; mj < mappedJoints[otherCId].size(); ++mj) {
                                            const JointId& currentMappedJoint = mappedJoints[otherCId][mj];
                                            const JointId& currentMappedJointConfidence = normalizedMappedJointConfidence[otherCId][mj];

                                            const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                            const Frame& fixedFrame1 = currentFixedFrames[currentFrameId[i][otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[i][otherCId] - 1];
                                            const Frame& fixedFrame2 = currentFixedFrames[currentFrameId[i][otherCId]];

                                            double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][otherCId];
                                            double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][otherCId];
                                            if (currentFrameId[i][otherCId] == 0) {
                                                fixedTime1 = fixedTime2;
                                            }

                                            const Transformation& fixedTransformation1 = fixedFrame1.transformation(currentMappedJoint);
                                            const Transformation& fixedTransformation2 = fixedFrame2.transformation(currentMappedJoint);

                                            double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                                            assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                                            Transformation fixedTnterpolatedTransformation = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);

                                            double similarity = internal::transformationSimilarityScore(fixedTnterpolatedTransformation, candidateTransformation2, fixedTime1, fixedTransformation1, fixedTime2, fixedTransformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                            assert(similarity >= 0 && similarity <= 1);

                                            jointFrameScore += currentMappedJointConfidence * candidateMappedJointConfidence * similarity;
                                        }

                                        numOtherAnimations++;
                                    }
                                }
                            }
#endif
                            if (numOtherAnimations > 0) {
                                jointFrameScore /= static_cast<double>(numOtherAnimations);
                            }

                            assert(jointFrameScore >= 0 && jointFrameScore <= 1);
                            frameScore += distanceWeight * jointFrameScore;
                        }

                        if (frameScore > bestKeyframeScore[i][cId]) {
                            bestKeyframeScore[i][cId] = frameScore;
                            bestKeyframeAnimation[i][cId] = candidateAId;
                            bestKeyframe[i][cId] = candidateFId;
                            bestKeyframeAlpha[i][cId] = 1.0;
                        }
                    }
                }
            }
        }
    }







    // ------------------------------------------ BLENDING DUPLICATES ------------------------------------------


    //Remove and blend duplicate frames
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& animationMode = animationModes[cId];
        if (animationMode == BLEND_ANIMATION_KEYFRAME) {
            for (Index i = 0; i < times.size(); ++i) {
                if (bestKeyframeAnimation[i][cId] == DUPLICATE_KEYFRAME_TO_BLEND || bestKeyframeAnimation[i][cId] == nvl::MAX_INDEX)
                    continue;

                Index maxScoreIndex = i;

                Index j = i+1;
                while (j < times.size() && bestKeyframeAnimation[i][cId] == bestKeyframeAnimation[j][cId] && bestKeyframe[i][cId] == bestKeyframe[j][cId]) {
                    if (bestKeyframeScore[j][cId] > bestKeyframeScore[i][cId]) {
                        maxScoreIndex = j;
                    }
                    j++;
                }

                if (i == 0 && j == times.size())
                    continue;

                if (i == 0) {
                    maxScoreIndex = 0;
                }
                if (j == times.size()) {
                    maxScoreIndex = times.size() - 1;
                }


                for (Index k = i; k < j; k++) {
                    if (k != maxScoreIndex) {
                        bestKeyframeAnimation[k][cId] = DUPLICATE_KEYFRAME_TO_BLEND;
                        bestKeyframe[k][cId] = DUPLICATE_KEYFRAME_TO_BLEND;
                    }
                }
            }
        }
    }




#ifdef KEYFRAME_SELECTION_VERBOSITY
    std::cout << std::endl << "Animation blending: " << std::endl;
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::cout << ">>>> " << i << " - Time: " << currentTime << std::endl;

        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& animationMode = animationModes[cId];
            if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                if (bestKeyframeAnimation[i][cId] != nvl::MAX_INDEX) {
                    std::cout << cId << " -> ";
                    if (bestKeyframeAnimation[i][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                        std::cout << "Blended" << std::endl;
                    }
                    else {
                        std::cout << "Animation: " << bestKeyframeAnimation[i][cId] << " - Frame: " << bestKeyframe[i][cId] << " - Alpha: " << bestKeyframeAlpha[i][cId] << std::endl;
                    }
                }
            }
        }
    }
#endif





    // ------------------------------------------ FILLING TRANSFORMATIONS ------------------------------------------

    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::vector<Transformation> blendedTransformations(targetSkeleton.jointNumber());

        //For each joint blend transformations
        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size(), Transformation::Identity());
            std::vector<double> weights(cluster.size(), 0.0);

            for (Index cId = 0; cId < cluster.size(); ++cId) {
                const Index& animationMode = animationModes[cId];
                const Index& animationId = animationIds[cId];

                //Calculate mapped joints and their weights for each entry
                std::vector<std::vector<JointId>> mappedJoints(cluster.size());
                std::vector<std::vector<double>> mappedJointConfidence(cluster.size());
                for (JointInfo jointInfo : jointInfos) {
                    mappedJoints[clusterMap[jointInfo.eId]].push_back(jointInfo.jId);
                    mappedJointConfidence[clusterMap[jointInfo.eId]].push_back(jointInfo.confidence);
                }

                //Avoid numerical errors
                if (!nvl::epsEqual(animationWeights[jId][cId], 0.0)) {
                    weights[cId] = animationWeights[jId][cId];

                    //Fixed
                    if (animationMode == BLEND_ANIMATION_FIXED) {
                        if (animationId == BLEND_ANIMATION_NONE) {
                            transformations[cId] = Transformation::Identity();
                        }
                        else {
                            const std::vector<Frame>& currentSelectedFrames = fixedFrames[cId];

                            const Frame& fixedFrame1 = currentSelectedFrames[currentFrameId[i][cId] == 0 ? currentSelectedFrames.size() - 1 : currentFrameId[i][cId] - 1];
                            const Frame& fixedFrame2 = currentSelectedFrames[currentFrameId[i][cId]];

                            double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][cId];
                            double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][cId];
                            if (currentFrameId[i][cId] == 0) {
                                fixedTime1 = fixedTime2;
                            }

                            Transformation fixedTransformation1 = internal::computeMappedTransformation(fixedFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation fixedTransformation2 = internal::computeMappedTransformation(fixedFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                            assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                            transformations[cId] = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);
                        }
                    }
                    else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                        if (bestKeyframeAnimation[i][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                            assert(animationMode == BLEND_ANIMATION_KEYFRAME);

                            Index prevIndex = i-1;
                            while (bestKeyframeAnimation[prevIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                                prevIndex--;
                            }

                            Index nextIndex = i+1;
                            while (bestKeyframeAnimation[nextIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                                nextIndex++;
                            }

                            assert(bestKeyframeAnimation[prevIndex][cId] != nvl::MAX_INDEX);
                            assert(bestKeyframe[prevIndex][cId] != nvl::MAX_INDEX);
                            assert(bestKeyframeAnimation[nextIndex][cId] != nvl::MAX_INDEX);
                            assert(bestKeyframe[nextIndex][cId] != nvl::MAX_INDEX);

                            assert(bestKeyframeAnimation[prevIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(bestKeyframe[prevIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(bestKeyframeAnimation[nextIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(bestKeyframe[nextIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);

                            const std::vector<Frame>& prevCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[prevIndex][cId]];
                            const std::vector<Frame>& nextCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[nextIndex][cId]];

                            const Frame& candidateFrame1 = prevCandidateFrames[bestKeyframe[prevIndex][cId]];
                            const Frame& candidateFrame2 = nextCandidateFrames[bestKeyframe[nextIndex][cId]];
                            double prevTime = times[prevIndex];
                            double nextTime = times[nextIndex];

                            Transformation candidateTransformation1 = internal::computeMappedTransformation(candidateFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation candidateTransformation2 = internal::computeMappedTransformation(candidateFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            double candidateAlpha = (currentTime - prevTime) / (nextTime - prevTime);
                            assert(candidateAlpha >= 0 && candidateAlpha <= 1);

                            transformations[cId] = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);
                        }
                        else if (bestKeyframeAnimation[i][cId] != nvl::MAX_INDEX) {
                            assert(bestKeyframeAnimation[i][cId] != nvl::MAX_INDEX);
                            assert(bestKeyframe[i][cId] != nvl::MAX_INDEX);

                            const std::vector<Frame>& currentCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[i][cId]];

                            const Frame& frame1 = currentCandidateFrames[bestKeyframe[i][cId] == 0 ? currentCandidateFrames.size() - 1 : bestKeyframe[i][cId] - 1];
                            const Frame& frame2 = currentCandidateFrames[bestKeyframe[i][cId]];

                            Transformation transformation1 = internal::computeMappedTransformation(frame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation transformation2 = internal::computeMappedTransformation(frame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            transformations[cId] = nvl::interpolateAffine(transformation1, transformation2, bestKeyframeAlpha[i][cId]);
                        }
                        else {
                            transformations[cId] = Transformation::Identity();
                        }
                    }
                }
            }

            nvl::normalize(weights);

            double sum = 0.0;
            for (const double& value : weights) {
                sum += value;
            }
            if (nvl::epsEqual(sum, 0.0)) {
                for (double& value : weights) {
                    value = 1.0 / weights.size();
                }
            }

            blendedTransformations[jId] = nvl::interpolateAffine(transformations, weights);
        }

        Frame targetFrame(currentTime, blendedTransformations);
        targetAnimation.addKeyframe(targetFrame);







        // ------------------------------------------ FILLING SELECTED KEYFRAMES ------------------------------------------

        //Filling selected keyframes
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Model* currentModel = data.entry(cluster[cId]).model;
            const Skeleton& currentSkeleton = currentModel->skeleton;

            std::vector<Transformation> clusterTransformations(currentSkeleton.jointNumber());

            //For each joint
            for (JointId jId = 0; jId < currentSkeleton.jointNumber(); ++jId) {
                const Index& animationMode = animationModes[cId];
                const Index& animationId = animationIds[cId];

                if (animationMode == BLEND_ANIMATION_FIXED) {
                    if (animationId == BLEND_ANIMATION_NONE) {
                        clusterTransformations[jId] = Transformation::Identity();
                    }
                    else {
                        const std::vector<Frame>& currentSelectedFrames = fixedFrames[cId];

                        const Frame& fixedFrame1 = currentSelectedFrames[currentFrameId[i][cId] == 0 ? currentSelectedFrames.size() - 1 : currentFrameId[i][cId] - 1];
                        const Frame& fixedFrame2 = currentSelectedFrames[currentFrameId[i][cId]];

                        double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][cId];
                        double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][cId];
                        if (currentFrameId[i][cId] == 0) {
                            fixedTime1 = fixedTime2;
                        }

                        const Transformation& transformation1 = fixedFrame1.transformation(jId);
                        const Transformation& transformation2 = fixedFrame2.transformation(jId);

                        double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                        assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                        clusterTransformations[jId] = nvl::interpolateAffine(transformation1, transformation2, fixedAlpha);
                    }
                }
                else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                    if (bestKeyframeAnimation[i][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                        Index prevIndex = i-1;
                        while (bestKeyframeAnimation[prevIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                            prevIndex--;
                        }

                        Index nextIndex = i+1;
                        while (bestKeyframeAnimation[nextIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                            nextIndex++;
                        }

                        assert(bestKeyframeAnimation[prevIndex][cId] != nvl::MAX_INDEX);
                        assert(bestKeyframe[prevIndex][cId] != nvl::MAX_INDEX);
                        assert(bestKeyframeAnimation[nextIndex][cId] != nvl::MAX_INDEX);
                        assert(bestKeyframe[nextIndex][cId] != nvl::MAX_INDEX);

                        assert(bestKeyframeAnimation[prevIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(bestKeyframe[prevIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(bestKeyframeAnimation[nextIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(bestKeyframe[nextIndex][cId] != DUPLICATE_KEYFRAME_TO_BLEND);

                        const std::vector<Frame>& prevCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[prevIndex][cId]];
                        const std::vector<Frame>& nextCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[nextIndex][cId]];

                        const Frame& candidateFrame1 = prevCandidateFrames[bestKeyframe[prevIndex][cId]];
                        const Frame& candidateFrame2 = nextCandidateFrames[bestKeyframe[nextIndex][cId]];
                        double prevTime = times[prevIndex];
                        double nextTime = times[nextIndex];

                        const Transformation& candidateTransformation1 = candidateFrame1.transformation(jId);
                        const Transformation& candidateTransformation2 = candidateFrame2.transformation(jId);

                        double candidateAlpha = (currentTime - prevTime) / (nextTime - prevTime);
                        assert(candidateAlpha >= 0 && candidateAlpha <= 1);

                        clusterTransformations[jId] = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);
                    }
                    else if (bestKeyframeAnimation[i][cId] != nvl::MAX_INDEX) {
                        assert(bestKeyframeAnimation[i][cId] != nvl::MAX_INDEX);
                        assert(bestKeyframe[i][cId] != nvl::MAX_INDEX);

                        const std::vector<Frame>& currentCandidateFrames = candidateFrames[cId][bestKeyframeAnimation[i][cId]];

                        const Frame& frame1 = currentCandidateFrames[bestKeyframe[i][cId] == 0 ? currentCandidateFrames.size() - 1 : bestKeyframe[i][cId] - 1];
                        const Frame& frame2 = currentCandidateFrames[bestKeyframe[i][cId]];

                        Transformation transformation1 = frame1.transformation(jId);
                        Transformation transformation2 = frame2.transformation(jId);

                        clusterTransformations[jId] = nvl::interpolateAffine(transformation1, transformation2, bestKeyframeAlpha[i][cId]);
                    }
                    else {
                        clusterTransformations[jId] = Transformation::Identity();
                    }
                }
            }

            Frame clusterFrame(currentTime, clusterTransformations);
            clusterAnimations[cId].addKeyframe(clusterFrame);
        }
    }




    targetAnimation.setName("Blended");
    Index targetAnimationId = targetModel->addAnimation(targetAnimation);
    resultAnimations.push_back(std::make_pair(entry.id, targetAnimationId));

    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];
        Model* currentModel = data.entry(cluster[cId]).model;

        clusterAnimations[cId].setName("Selected keyframe");
        Index clusterAnimationId = currentModel->addAnimation(clusterAnimations[cId]);
        resultAnimations.push_back(std::make_pair(eId, clusterAnimationId));
    }
}

namespace internal {

template<class Frame, class JointId>
typename Frame::Transformation computeMappedTransformation(
        const Frame& frame,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence)
{
    typedef typename Frame::Transformation Transformation;
    typedef nvl::Index Index;
    std::vector<Transformation> mappedTransformations;
    std::vector<double> mappedTransformationsWeights;

    double confidenceSum = 0.0;
    for (Index i = 0; i < mappedJoints.size(); i++) {
        const Transformation& t = frame.transformation(mappedJoints[i]);
        mappedTransformations.push_back(t);
        mappedTransformationsWeights.push_back(mappedJointConfidence[i]);
        confidenceSum += mappedJointConfidence[i];
    }

    if (nvl::epsEqual(confidenceSum, 1.0)) {
        confidenceSum = 1.0;
    }
//    else {
//        nvl::normalize(mappedTransformationsWeights);
//    }
    else if (confidenceSum < 1.0) {
        mappedTransformations.push_back(Transformation::Identity());
        mappedTransformationsWeights.push_back(1.0 - confidenceSum);
    }
    else {
        assert(confidenceSum > 1.0);
        nvl::normalize(mappedTransformationsWeights);
    }

    Transformation transformation = nvl::interpolateAffine(mappedTransformations, mappedTransformationsWeights);
    return transformation;
}

template<class T>
double transformationSimilarityScore(
        const T& fixedInterpolatedTransformation,
        const T& candidateInterpolatedTransformation,
        const double& fixedTime1,
        const T& fixedTransformation1,
        const double& fixedTime2,
        const T& fixedTransformation2,
        const double& candidateTime1,
        const T& candidateTransformation1,
        const double& candidateTime2,
        const T& candidateTransformation2)
{
    const double globalWeight = 1.0;
    const double localWeight = 0.0;


    nvl::Quaterniond fixedInterpolatedQuaternion(fixedInterpolatedTransformation.rotation());
    nvl::Quaterniond candidateInterpolatedQuaternion(candidateInterpolatedTransformation.rotation());
    fixedInterpolatedQuaternion.normalize();
    candidateInterpolatedQuaternion.normalize();

    double globalScore = 0.0;
    if (globalWeight > 0.0) {
        globalScore = nvl::abs(fixedInterpolatedQuaternion.dot(candidateInterpolatedQuaternion));
        if (nvl::epsEqual(globalScore, 1.0))
            globalScore = 1.0;
        else if (nvl::epsEqual(globalScore, 0.0))
            globalScore = 0.0;
        assert(globalScore >= 0.0 && globalScore <= 1.0);
    }



    double localScore = 0.0;
    if (localWeight > 0.0) {
        if (candidateTime1 == candidateTime2 && fixedTime1 == fixedTime2) {
            localScore = 1.0;
        }
        else if (candidateTime1 != candidateTime2 && fixedTime1 != fixedTime2) {
            assert(candidateTime1 < candidateTime2);
            assert(fixedTime1 < fixedTime2);

            nvl::Quaterniond fixedQuaternion1(fixedTransformation1.rotation());
            nvl::Quaterniond fixedQuaternion2(fixedTransformation2.rotation());
            nvl::Quaterniond candidateQuaternion1(candidateTransformation1.rotation());
            nvl::Quaterniond candidateQuaternion2(candidateTransformation2.rotation());
            fixedQuaternion1.normalize();
            fixedQuaternion2.normalize();
            candidateQuaternion1.normalize();
            candidateQuaternion2.normalize();

            nvl::Rotation3d localFixedRotation(fixedQuaternion2 * fixedQuaternion1.inverse());
            nvl::Rotation3d localCandidateRotation(candidateQuaternion2 * candidateQuaternion1.inverse());

            double localFixedAngle = localFixedRotation.angle() / (fixedTime2 - fixedTime1);
            double localCandidateAngle = localCandidateRotation.angle() / (candidateTime2 - candidateTime1);

            nvl::Quaterniond localFixedQuaternion(nvl::Rotation3d(localFixedAngle, localFixedRotation.axis()));
            nvl::Quaterniond localCandidateQuaternion(nvl::Rotation3d(localCandidateAngle, localCandidateRotation.axis()));
            localFixedQuaternion.normalize();
            localCandidateQuaternion.normalize();

            localScore = 0.0;
            localScore = nvl::abs(localFixedQuaternion.dot(localCandidateQuaternion));
            if (nvl::epsEqual(localScore, 1.0))
                localScore = 1.0;
            else if (nvl::epsEqual(localScore, 0.0))
                localScore = 0.0;
            assert(localScore >= 0.0 && localScore <= 1.0);
        }
    }



    return globalWeight * globalScore +
           localWeight * localScore;
}

inline double computeDistanceWeight(const double distance) {
    const unsigned int minDistance = 2;

    double distanceWeight = 1.0;
    if (distance > minDistance) {
        if (distance > 8) {
            distanceWeight = 0.0;
        }
        else {
            double exponent = -(static_cast<double>(distance) - static_cast<double>(minDistance));
            distanceWeight = nvl::pow(2.0, exponent);
        }
    }

    return distanceWeight;
}

}
}
