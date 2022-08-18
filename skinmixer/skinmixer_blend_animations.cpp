#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_function.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>
#include <nvl/math/numeric_limits.h>

#include <nvl/models/algorithms/animation_poses.h>
#include <nvl/models/algorithms/animation_blend.h>
#include <nvl/models/algorithms/animation_smoothing.h>
#include <nvl/models/algorithms/animation_transformations.h>
#include <nvl/models/algorithms/skeleton_bind_pose.h>

#define KEYFRAME_SELECTION_VERBOSITY
#define DUPLICATE_KEYFRAME_TO_BLEND nvl::NULL_ID - 1
#define SIMPLE_ANIMATION_NAME

#ifdef KEYFRAME_SELECTION_VERBOSITY
#include <iostream>
#endif

namespace skinmixer {

namespace internal {

template<class Frame, class JointId>
typename Frame::Transformation computeMappedTransformation(
        const Frame& frame,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence);


template<class Frame, class JointId>
void computeWindow(
        const std::vector<Frame>& frames,
        const nvl::Index& fId,
        const unsigned int& windowSize,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence,
        std::vector<nvl::Quaterniond>& quaternionWindow,
        std::vector<nvl::Translation3d>& translationWindow);

inline double windowSimilarity(
        const std::vector<nvl::Quaterniond>& globalQ1,
        const std::vector<nvl::Quaterniond>& globalQ2,
        const std::vector<nvl::Quaterniond>& localQ1,
        const std::vector<nvl::Quaterniond>& localQ2,
        const std::vector<nvl::Translation3d>& globalT1,
        const std::vector<nvl::Translation3d>& globalT2,
        const std::vector<nvl::Translation3d>& localT1,
        const std::vector<nvl::Translation3d>& localT2,
        const std::vector<double>& windowWeights,
        const double& rotationWeight,
        const double& globalWeight,
        const double& localWeight,
        const double& globalDerivativeWeight,
        const double& localDerivativeWeight);

double quaternionSimilarity(
        const nvl::Quaterniond& q1,
        const nvl::Quaterniond& q2);

double translationSimilarity(
        const nvl::Translation3d& t1,
        const nvl::Translation3d& t2);

template<class S>
inline std::vector<double> computeDistanceWeights(const S& skeleton, const std::vector<typename S::JointId>& seeds);

}

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry)
{
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    NVL_SUPPRESS_UNUSEDVARIABLE(data);

    Model* targetModel = resultEntry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;

    std::vector<Index>& animationsIds = resultEntry.blendingAnimationIds;
    std::vector<Index>& animationsModes = resultEntry.blendingAnimationModes;
    std::vector<std::vector<double>>& animationWeights = resultEntry.blendingAnimationWeights;
    std::vector<double>& animationSpeeds = resultEntry.blendingAnimationSpeeds;

    animationWeights.resize(targetSkeleton.jointNumber(), std::vector<double>(cluster.size(), 0.0));
    animationSpeeds.resize(cluster.size(), 1.0);

    std::vector<Index> clusterMap = nvl::inverseFunction(cluster);
    for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
        const std::vector<JointInfo>& jointInfos = resultEntry.birth.joint[jId];

        int numOneConfidence = 0;

        for (JointInfo jointInfo : jointInfos) {
            assert(jointInfo.jId != nvl::NULL_ID);
            assert(jointInfo.eId != nvl::NULL_ID);
            assert(clusterMap[jointInfo.eId] != nvl::NULL_ID);

            const Index& cId = clusterMap[jointInfo.eId];

            if (jointInfo.confidence == 1.0) {
                animationWeights[jId][cId] = 1.0;
                numOneConfidence++;
            }
        }

        if (numOneConfidence > 1) {
            Index bestCId = nvl::NULL_ID;

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::NULL_ID);
                assert(jointInfo.eId != nvl::NULL_ID);
                assert(clusterMap[jointInfo.eId] != nvl::NULL_ID);

                const Index& cId = clusterMap[jointInfo.eId];

                for (Index aId = 0; aId < data.actionNumber(); ++aId) {
                    const Action& action = data.action(aId);

                    if (action.operation == OperationType::REPLACE && action.entry2 == jointInfo.eId && action.joint2 == jointInfo.jId) {
                        bestCId = cId;
                    }
                    else if (action.operation == OperationType::ATTACH && action.entry1 == jointInfo.eId && action.joint1 == jointInfo.jId) {
                        bestCId = cId;
                    }
                }
            }

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::NULL_ID);
                assert(jointInfo.eId != nvl::NULL_ID);
                assert(clusterMap[jointInfo.eId] != nvl::NULL_ID);

                const Index& cId = clusterMap[jointInfo.eId];

                if (bestCId != cId) {
                    animationWeights[jId][cId] = 0.0;
                }
            }
        }

        nvl::normalize(animationWeights[jId]);
    }

    animationsModes.resize(cluster.size(), BLEND_ANIMATION_FIXED);
    animationsIds.resize(cluster.size(), BLEND_ANIMATION_ANY);
}

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        std::vector<std::pair<nvl::Index, nvl::Index>>& resultAnimations,
        const double& samplingFPS,
        const double& rotationWeight,
        const double& globalWeight,
        const double& localWeight,
        const double& globalDerivativeWeight,
        const double& localDerivativeWeight,
        const unsigned int& windowSize,
        const double& mainFrameWeight,
        const unsigned int& smoothingIterations,
        const double& smoothingThreshold)
{
    typedef typename nvl::Index Index;

    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Skeleton::Transformation SkeletonTransformation;

    typedef typename Model::Animation Animation;
    typedef typename Animation::Frame Frame;
    typedef typename Animation::Transformation Transformation;

    typedef nvl::Quaterniond Quaterniond;
    typedef nvl::Translation3d Translation3d;

    //Get data
    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;
    std::vector<Index>& cluster = entry.birth.entries;
    const std::vector<Index>& animationIds = entry.blendingAnimationIds;
    const std::vector<Index>& animationModes = entry.blendingAnimationModes;
    const std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;
    const std::vector<double>& animationSpeeds = entry.blendingAnimationSpeeds;

    //Name of the animation
    std::string animationName;

    //Get max duration of fixed animations
    double maxFixedDuration = 0;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;

        Index animationMode = animationModes[cId];
        Index animationId = animationIds[cId];

        if (animationMode == BLEND_ANIMATION_FIXED && animationId != BLEND_ANIMATION_ANY) {
            const Animation& currentAnimation = currentModel->animation(animationId);
            maxFixedDuration = std::max(currentAnimation.duration(), maxFixedDuration);
        }
    }

    //Cluster to entry map
    std::vector<Index> clusterMap = nvl::inverseFunction(cluster);

    //Resulting animations
    Animation targetAnimation;
    std::vector<Animation> clusterAnimations(cluster.size());

    //Data for fixed and candidate frames for each cluster
    std::vector<std::vector<Frame>> localFixedFrames(cluster.size());
    std::vector<std::vector<std::vector<Frame>>> localCandidateFrames(cluster.size());

    //Target local bind pose
    std::vector<SkeletonTransformation> targetLocalBindPose = nvl::skeletonLocalBindPose(targetSkeleton);

    //Fill candidate and fixed frames
    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        //Inverse local bind pose of the current skeleton
        std::vector<SkeletonTransformation> currentLocalInverseBindPose = nvl::skeletonLocalInverseBindPose(currentSkeleton);

        //Animation data
        Index animationMode = animationModes[cId];
        Index animationId = animationIds[cId];

        //Fixed mode
        if (animationMode == BLEND_ANIMATION_FIXED) {
            if (animationId != BLEND_ANIMATION_ANY) {
                const Animation& currentAnimation = currentModel->animation(animationId);

#ifndef SIMPLE_ANIMATION_NAME
                animationName += " - " + currentModel->name + "@" + currentAnimation.name().c_str() + " (F)";
#else
                animationName += currentModel->name + "@" + currentAnimation.name().c_str();
#endif

                for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                    Frame frame = currentAnimation.keyframe(fId);

                    nvl::animationFrameLocalFromGlobal(currentSkeleton, frame);
                    nvl::animationFrameApplyTransformation(frame, currentLocalInverseBindPose);

                    localFixedFrames[cId].push_back(frame);
                }
            }
        }
        //Best keyframe or best sliding mode
        else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
            localCandidateFrames[cId].resize(currentModel->animationNumber());

            if (animationMode == BLEND_ANIMATION_KEYFRAME) {
#ifndef SIMPLE_ANIMATION_NAME
                animationName += " - " + currentModel->name + " (K)";
#endif
            }
            else {
                assert(animationMode == BLEND_ANIMATION_LOOP);
#ifndef SIMPLE_ANIMATION_NAME
                animationName += " - " + currentModel->name + " (L)";
#endif
            }

            for (Index aId = 0; aId < currentModel->animationNumber(); ++aId) {
                if (animationId == BLEND_ANIMATION_ANY|| animationId == aId) {
                    assert(animationId == BLEND_ANIMATION_ANY || (animationId != nvl::NULL_ID && animationId < currentModel->animationNumber()));

                    const Animation& currentAnimation = currentModel->animation(aId);
                    for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                        Frame frame = currentAnimation.keyframe(fId);

                        nvl::animationFrameLocalFromGlobal(currentSkeleton, frame);
                        nvl::animationFrameApplyTransformation(frame, currentLocalInverseBindPose);

                        localCandidateFrames[cId][aId].push_back(frame);
                    }
                }
            }
        }

        //Blend frames to a given number of fps
        nvl::animationFrameBlend(localFixedFrames[cId], samplingFPS, animationSpeeds[cId], false);

        for (Index aId = 0; aId < localCandidateFrames[cId].size(); aId++) {
            nvl::animationFrameBlend(localCandidateFrames[cId][aId], samplingFPS, animationSpeeds[cId], false);
        }

        //Add times of fixed frames
        for (Index fId = 0; fId < localFixedFrames[cId].size(); ++fId) {
            times.push_back(localFixedFrames[cId][fId].time());
        }
    }

    //At least one time entry
    if (times.empty())
        times.push_back(0.0);

    //Sort and remove duplicates
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());



    //Calculate window weights
    const double sigma = 1.5; //Gaussian sigma

    std::vector<double> windowWeights(windowSize * 2 + 1, 0.0);
    for (int i = 0; i < static_cast<int>(windowWeights.size()); ++i) {
        if (i == static_cast<int>(windowSize))
            continue;

        windowWeights[i] =
            std::exp(-0.5 * (std::pow((i - static_cast<int>(windowSize))/sigma, 2.0))) / (2 * M_PI * sigma * sigma);
    }

    nvl::normalize(windowWeights);

    for (int i = 0; i < static_cast<int>(windowWeights.size()); ++i) {
        if (i == static_cast<int>(windowSize)) {
            windowWeights[i] = mainFrameWeight;
        }
        else {
            windowWeights[i] *= 1.0 - mainFrameWeight;
        }
    }

    nvl::normalize(windowWeights);

#ifdef KEYFRAME_SELECTION_VERBOSITY
    std::cout << std::endl << "--------- B" << animationName << "---------" << std::endl;
    std::cout << "Window weights: ";
    for (Index wId = 0; wId < windowWeights.size(); wId++) {
        std::cout << windowWeights[wId] << " ";
    }
    std::cout << std::endl;
#endif



    //Compute global frames
    std::vector<std::vector<Frame>> globalFixedFrames = localFixedFrames;
    std::vector<std::vector<std::vector<Frame>>> globalCandidateFrames = localCandidateFrames;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Model* currentModel = data.entry(cluster[cId]).model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        //Calculate global frames
        nvl::animationFrameGlobalFromLocal(currentSkeleton, globalFixedFrames[cId]);
        for (Index aId = 0; aId < localCandidateFrames[cId].size(); aId++) {
            nvl::animationFrameGlobalFromLocal(currentSkeleton, globalCandidateFrames[cId][aId]);
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
        bestKeyframeAnimation[i].resize(cluster.size(), nvl::NULL_ID);
        bestKeyframe[i].resize(cluster.size(), nvl::NULL_ID);
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
            const std::vector<Frame>& currentLocalFixedFrames = localFixedFrames[cId];
            while (currentFrameId[i][cId] < currentLocalFixedFrames.size() && currentLocalFixedFrames[currentFrameId[i][cId]].time() + currentTimeOffset[i][cId] <= currentTime) {
                ++currentFrameId[i][cId];

                if (currentFrameId[i][cId] >= currentLocalFixedFrames.size()) {
                    currentTimeOffset[i][cId] += currentLocalFixedFrames[currentLocalFixedFrames.size() - 1].time();
                    currentFrameId[i][cId] = 0;
                }
            }
        }
    }

    //Joint distance from the merge joints
    std::vector<double> jointDistanceWeight = internal::computeDistanceWeights(targetSkeleton, entry.birth.mergeJoints);


    // ------------------------------------------ BEST LOOP ------------------------------------------

    //Compute best frames for select best keyframe animation
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index animationMode = animationModes[cId];
        Index animationId = animationIds[cId];

        if (animationMode == BLEND_ANIMATION_LOOP) {
            Index eId = cluster[cId];
            const Entry& currentEntry = data.entry(eId);
            const Model* currentModel = currentEntry.model;

            std::vector<Index> candidateAnimations;
            if (animationId == BLEND_ANIMATION_ANY) {
                for (Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                    candidateAnimations.push_back(candidateAId);
                }
            }
            else {
                candidateAnimations.push_back(animationId);
            }

            double bestLoopScore = nvl::minLimitValue<double>();
            Index bestLoopAnimationId = nvl::NULL_ID;
            Index bestLoopStartingFrame = nvl::NULL_ID;
            std::vector<double> bestLoopScoreSingle;

            for (const Index& candidateAId : candidateAnimations) {
                const std::vector<Frame>& currentGlobalCandidateFrames = globalCandidateFrames[cId][candidateAId];
                const std::vector<Frame>& currentLocalCandidateFrames = localCandidateFrames[cId][candidateAId];

                for (Index startingFId = 0; startingFId < currentGlobalCandidateFrames.size(); startingFId++) {
                    const double startingTime = currentGlobalCandidateFrames[startingFId].time();

                    Index loopFrameId = startingFId;
                    double loopFrameOffset = -startingTime;

                    double loopScore = 0.0;
                    std::vector<double> loopScoreSingle(times.size(), 0.0);

                    for (Index i = 0; i < times.size(); ++i) {
                        const double& currentTime = times[i];

                        while (loopFrameId < currentGlobalCandidateFrames.size() && currentGlobalCandidateFrames[loopFrameId].time() + loopFrameOffset <= currentTime) {
                            ++loopFrameId;

                            if (loopFrameId >= currentGlobalCandidateFrames.size()) {
                                loopFrameOffset += currentGlobalCandidateFrames[currentGlobalCandidateFrames.size() - 1].time();
                                loopFrameId = 0;
                            }
                        }

                        //For each joint
                        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                            unsigned int numOtherAnimations = 0;
                            double jointFrameScore = 0.0;

                            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                            const double& distanceWeight = jointDistanceWeight[jId];
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

                            std::vector<Quaterniond> candidateGlobalQuaternionWindow;
                            std::vector<Translation3d> candidateGlobalTranslationWindow;
                            if (globalWeight > 0.0 || globalDerivativeWeight > 0.0) {
                                internal::computeWindow(currentGlobalCandidateFrames, loopFrameId, windowSize, mappedJoints[cId], mappedJointConfidence[cId], candidateGlobalQuaternionWindow, candidateGlobalTranslationWindow);
                            }

                            std::vector<Quaterniond> candidateLocalQuaternionWindow;
                            std::vector<Translation3d> candidateLocalTranslationWindow;
                            if (localWeight > 0.0 || localDerivativeWeight > 0.0) {
                                internal::computeWindow(currentLocalCandidateFrames, loopFrameId, windowSize, mappedJoints[cId], mappedJointConfidence[cId], candidateLocalQuaternionWindow, candidateLocalTranslationWindow);
                            }

                            for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                Index otherAnimationMode = animationModes[otherCId];
                                Index otherAnimationId = animationIds[otherCId];
                                if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_ANY) {
                                    const std::vector<Frame>& currentGlobalFixedFrames = globalFixedFrames[otherCId];
                                    const std::vector<Frame>& currentLocalFixedFrames = localFixedFrames[otherCId];

                                    std::vector<Quaterniond> fixedGlobalQuaternionWindow;
                                    std::vector<Translation3d> fixedGlobalTranslationWindow;
                                    if (globalWeight > 0.0 || globalDerivativeWeight > 0.0) {
                                        internal::computeWindow(currentGlobalFixedFrames, currentFrameId[i][otherCId], windowSize, mappedJoints[otherCId], mappedJointConfidence[otherCId], fixedGlobalQuaternionWindow, fixedGlobalTranslationWindow);
                                    }
                                    std::vector<Quaterniond> fixedLocalQuaternionWindow;
                                    std::vector<Translation3d> fixedLocalTranslationWindow;
                                    if (localWeight > 0.0 || localDerivativeWeight > 0.0) {
                                        internal::computeWindow(currentLocalFixedFrames, currentFrameId[i][otherCId], windowSize, mappedJoints[otherCId], mappedJointConfidence[otherCId], fixedLocalQuaternionWindow, fixedLocalTranslationWindow);
                                    }

                                    double similarity = internal::windowSimilarity(
                                                candidateGlobalQuaternionWindow, fixedGlobalQuaternionWindow, candidateLocalQuaternionWindow, fixedLocalQuaternionWindow,
                                                candidateGlobalTranslationWindow, fixedGlobalTranslationWindow, candidateLocalTranslationWindow, fixedLocalTranslationWindow,
                                                windowWeights, rotationWeight, globalWeight, localWeight, globalDerivativeWeight, localDerivativeWeight);

                                    assert(similarity >= 0 && similarity <= 1);

                                    jointFrameScore += similarity;

                                    numOtherAnimations++;
                                }
                            }

                            if (numOtherAnimations > 0) {
                                jointFrameScore /= static_cast<double>(numOtherAnimations);
                            }
                            assert(jointFrameScore >= 0 && jointFrameScore <= 1);

                            loopScoreSingle[i] = distanceWeight * jointFrameScore;
                            assert(loopScoreSingle[i] >= 0 && loopScoreSingle[i]<= 1);

                            loopScore += loopScoreSingle[i];
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

            if (bestLoopAnimationId != nvl::NULL_ID) {
                const std::vector<Frame>& currentGlobalCandidateFrames = globalCandidateFrames[cId][bestLoopAnimationId];

                const double startingTime = currentGlobalCandidateFrames[bestLoopStartingFrame].time();

                Index loopFrameId = bestLoopStartingFrame;
                double loopFrameOffset = -startingTime;
                for (Index i = 0; i < times.size(); ++i) {
                    Index& bestAId = bestKeyframeAnimation[i][cId];
                    Index& bestFId = bestKeyframe[i][cId];
                    double& bestAlpha = bestKeyframeAlpha[i][cId];
                    double& bestScore = bestKeyframeScore[i][cId];

                    const double& currentTime = times[i];

                    while (loopFrameId < currentGlobalCandidateFrames.size() && currentGlobalCandidateFrames[loopFrameId].time() + loopFrameOffset <= currentTime) {
                        ++loopFrameId;

                        if (loopFrameId >= currentGlobalCandidateFrames.size()) {
                            loopFrameOffset += currentGlobalCandidateFrames[currentGlobalCandidateFrames.size() - 1].time();
                            loopFrameId = 0;
                        }
                    }

                    bestScore = bestLoopScoreSingle[i];
                    bestAId = bestLoopAnimationId;
                    bestFId = loopFrameId;
                    bestAlpha = 1.0;
                }
            }
        }
    }




    // ------------------------------------------ BEST KEYFRAMES ------------------------------------------

    //For each time entry find the best keyframes
    for (Index i = 0; i < times.size(); ++i) {
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            Index animationMode = animationModes[cId];
            Index animationId = animationIds[cId];

            Index& bestAId = bestKeyframeAnimation[i][cId];
            Index& bestFId = bestKeyframe[i][cId];
            double& bestAlpha = bestKeyframeAlpha[i][cId];
            double& bestScore = bestKeyframeScore[i][cId];

            const Model* currentModel = data.entry(cluster[cId]).model;

            if (animationMode == BLEND_ANIMATION_KEYFRAME) {
                std::vector<Index> candidateAnimations;
                if (animationId == BLEND_ANIMATION_ANY) {
                    for (Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                        candidateAnimations.push_back(candidateAId);
                    }
                }
                else {
                    candidateAnimations.push_back(animationId);
                }

                for (const Index& candidateAId : candidateAnimations) {
                    const std::vector<Frame>& currentGlobalCandidateFrames = globalCandidateFrames[cId][candidateAId];
                    const std::vector<Frame>& currentLocalCandidateFrames = localCandidateFrames[cId][candidateAId];

                    for (Index candidateFId = 0; candidateFId < currentGlobalCandidateFrames.size(); candidateFId++) {
                        double frameScore = 0.0;

                        //For each joint
                        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                            unsigned int numOtherAnimations = 0;
                            double jointFrameScore = 0.0;

                            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                            const double& distanceWeight = jointDistanceWeight[jId];
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
                            std::vector<Quaterniond> candidateGlobalQuaternionWindow;
                            std::vector<Translation3d> candidateGlobalTranslationWindow;
                            if (globalWeight > 0.0 || globalDerivativeWeight > 0.0) {
                                internal::computeWindow(currentGlobalCandidateFrames, candidateFId, windowSize, mappedJoints[cId], mappedJointConfidence[cId], candidateGlobalQuaternionWindow, candidateGlobalTranslationWindow);
                            }

                            std::vector<Quaterniond> candidateLocalQuaternionWindow;
                            std::vector<Translation3d> candidateLocalTranslationWindow;
                            if (localWeight > 0.0 || localDerivativeWeight > 0.0) {
                                internal::computeWindow(currentLocalCandidateFrames, candidateFId, windowSize, mappedJoints[cId], mappedJointConfidence[cId], candidateLocalQuaternionWindow, candidateLocalTranslationWindow);
                            }

                            for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                Index otherAnimationMode = animationModes[otherCId];
                                Index otherAnimationId = animationIds[otherCId];
                                if (otherAnimationMode == BLEND_ANIMATION_FIXED && otherAnimationId != BLEND_ANIMATION_ANY) {
                                    const std::vector<Frame>& currentGlobalFixedFrames = globalFixedFrames[otherCId];
                                    const std::vector<Frame>& currentLocalFixedFrames = localFixedFrames[otherCId];

                                    std::vector<Quaterniond> fixedGlobalQuaternionWindow;
                                    std::vector<Translation3d> fixedGlobalTranslationWindow;
                                    if (globalWeight > 0.0 || globalDerivativeWeight > 0.0) {
                                        internal::computeWindow(currentGlobalFixedFrames, currentFrameId[i][otherCId], windowSize, mappedJoints[otherCId], mappedJointConfidence[otherCId], fixedGlobalQuaternionWindow, fixedGlobalTranslationWindow);
                                    }
                                    std::vector<Quaterniond> fixedLocalQuaternionWindow;
                                    std::vector<Translation3d> fixedLocalTranslationWindow;
                                    if (localWeight > 0.0 || localDerivativeWeight > 0.0) {
                                        internal::computeWindow(currentLocalFixedFrames, currentFrameId[i][otherCId], windowSize, mappedJoints[otherCId], mappedJointConfidence[otherCId], fixedLocalQuaternionWindow, fixedLocalTranslationWindow);
                                    }

                                    double similarity = internal::windowSimilarity(
                                                candidateGlobalQuaternionWindow, fixedGlobalQuaternionWindow, candidateLocalQuaternionWindow, fixedLocalQuaternionWindow,
                                                candidateGlobalTranslationWindow, fixedGlobalTranslationWindow, candidateLocalTranslationWindow, fixedLocalTranslationWindow,
                                                windowWeights, rotationWeight, globalWeight, localWeight, globalDerivativeWeight, localDerivativeWeight);

                                    assert(similarity >= 0 && similarity <= 1);


                                    jointFrameScore += similarity;

                                    numOtherAnimations++;
                                }
                            }

                            if (numOtherAnimations > 0) {
                                jointFrameScore /= static_cast<double>(numOtherAnimations);
                            }
                            assert(jointFrameScore >= 0 && jointFrameScore <= 1);
                            
                            frameScore += distanceWeight * jointFrameScore;
                        }
                        assert(frameScore >= 0 && frameScore <= 1);

                        if (frameScore > bestKeyframeScore[i][cId]) {
                            bestAId = candidateAId;
                            bestFId = candidateFId;
                            bestAlpha = 1.0;
                            bestScore = frameScore;
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
                if (bestKeyframeAnimation[i][cId] == DUPLICATE_KEYFRAME_TO_BLEND || bestKeyframeAnimation[i][cId] == nvl::NULL_ID)
                    continue;

                const Index& bestAId = bestKeyframeAnimation[i][cId];
                const Index& bestFId = bestKeyframe[i][cId];

                Index maxScoreIndex = i;

                Index j = i+1;
                while (j < times.size() && bestAId == bestKeyframeAnimation[j][cId] && bestFId == bestKeyframe[j][cId] && nvl::epsEqual(bestKeyframeAlpha[j][cId], bestKeyframeAlpha[i][cId], 1e2)) {
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

    std::cout.precision(4);
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::cout << std::fixed << currentTime << " (" << i << ") ->\t";

        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& animationMode = animationModes[cId];

            if (cId > 0) {
                std::cout << "\t";
            }

            if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                const Index& bestAId = bestKeyframeAnimation[i][cId];
                const Index& bestFId = bestKeyframe[i][cId];
                const double& bestScore = bestKeyframeScore[i][cId];

                if (bestAId != nvl::NULL_ID) {
                    if (bestAId == DUPLICATE_KEYFRAME_TO_BLEND) {
                        std::cout << "B";
                    }
                    else {
                        std::cout << "(" << bestAId << ", " << bestFId << ", " << bestScore << ")";
                    }
                }
            }
            else {
                std::cout << "F";
            }
        }

        std::cout << std::endl;
    }
#endif


    // ------------------------------------------ DETERMINING SMOOTHING ALPHAS ------------------------------------------

    std::vector<std::vector<double>> keyframeSmoothAlpha;

    if (smoothingIterations > 0) {
        keyframeSmoothAlpha.resize(times.size(), std::vector<double>(targetSkeleton.jointNumber(), 0.0));

        for (Index i = 0; i < times.size(); ++i) {
            for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                std::vector<double> clusterKeyframeWeight(cluster.size(), 0.0);
                std::vector<double> weights(cluster.size(), 0.0);

                for (Index cId = 0; cId < cluster.size(); ++cId) {
                    const Index& animationMode = animationModes[cId];

                    //Avoid numerical errors
                    if (!nvl::epsEqual(animationWeights[jId][cId], 0.0)) {
                        weights[cId] = animationWeights[jId][cId];

                        //Fixed
                        if (animationMode == BLEND_ANIMATION_FIXED) {
                            clusterKeyframeWeight[cId] = 1.0;
                        }
                        else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                            const Index& bestAId = bestKeyframeAnimation[i][cId];
                            const double& bestScore = bestKeyframeScore[i][cId];

                            if (bestAId == DUPLICATE_KEYFRAME_TO_BLEND) {
                                clusterKeyframeWeight[cId] = 0.0;
                            }
                            else if (bestAId != nvl::NULL_ID) {
                                assert(bestAId != nvl::NULL_ID);
                                assert(bestScore < nvl::maxLimitValue<double>());

                                clusterKeyframeWeight[cId] = bestScore;
                            }
                            else {
                                clusterKeyframeWeight[cId] = 0.0;
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

                keyframeSmoothAlpha[i][jId] = nvl::interpolateLinear(clusterKeyframeWeight, weights);
                if (keyframeSmoothAlpha[i][jId] > smoothingThreshold) {
                    keyframeSmoothAlpha[i][jId] = 1.0;
                }
            }
        }
    }




    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];

        // ------------------------------------------ FILLING TRANSFORMATIONS ------------------------------------------

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
                        if (animationId == BLEND_ANIMATION_ANY) {
                            transformations[cId] = Transformation::Identity();
                        }
                        else {
                            const std::vector<Frame>& currentLocalSelectedFrames = localFixedFrames[cId];

                            const Frame& fixedFrame1 = currentLocalSelectedFrames[currentFrameId[i][cId] == 0 ? currentLocalSelectedFrames.size() - 1 : currentFrameId[i][cId] - 1];
                            const Frame& fixedFrame2 = currentLocalSelectedFrames[currentFrameId[i][cId]];

                            Transformation fixedTransformation1 = internal::computeMappedTransformation(fixedFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation fixedTransformation2 = internal::computeMappedTransformation(fixedFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][cId];
                            double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][cId];
                            if (currentFrameId[i][cId] == 0) {
                                fixedTime1 = fixedTime2;
                            }

                            double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                            assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                            transformations[cId] = nvl::interpolateAffine(fixedTransformation1, fixedTransformation2, fixedAlpha);
                        }
                    }
                    else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                        const Index& bestAId = bestKeyframeAnimation[i][cId];
                        const Index& bestFId = bestKeyframe[i][cId];

                        if (bestAId == DUPLICATE_KEYFRAME_TO_BLEND) {
                            assert(animationMode == BLEND_ANIMATION_KEYFRAME);

                            Index prevIndex = i-1;
                            while (bestKeyframeAnimation[prevIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                                prevIndex--;
                            }

                            Index nextIndex = i+1;
                            while (bestKeyframeAnimation[nextIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                                nextIndex++;
                            }

                            const Index& prevAId = bestKeyframeAnimation[prevIndex][cId];
                            const Index& prevFId = bestKeyframe[prevIndex][cId];
                            const Index& nextAId = bestKeyframeAnimation[nextIndex][cId];
                            const Index& nextFId = bestKeyframe[nextIndex][cId];

                            assert(prevAId != nvl::NULL_ID);
                            assert(prevFId != nvl::NULL_ID);
                            assert(nextAId != nvl::NULL_ID);
                            assert(nextFId != nvl::NULL_ID);

                            assert(prevAId != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(prevFId != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(nextAId != DUPLICATE_KEYFRAME_TO_BLEND);
                            assert(nextFId != DUPLICATE_KEYFRAME_TO_BLEND);

                            const std::vector<Frame>& prevCandidateFrames = localCandidateFrames[cId][prevAId];
                            const std::vector<Frame>& nextCandidateFrames = localCandidateFrames[cId][nextAId];

                            const Frame& candidateFrame1 = prevCandidateFrames[prevFId];
                            const Frame& candidateFrame2 = nextCandidateFrames[nextFId];
                            double prevTime = times[prevIndex];
                            double nextTime = times[nextIndex];

                            Transformation candidateTransformation1 = internal::computeMappedTransformation(candidateFrame1, mappedJoints[cId], mappedJointConfidence[cId]);
                            Transformation candidateTransformation2 = internal::computeMappedTransformation(candidateFrame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            double candidateAlpha = (currentTime - prevTime) / (nextTime - prevTime);
                            assert(candidateAlpha >= 0 && candidateAlpha <= 1);

                            transformations[cId] = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);
                        }
                        else if (bestAId != nvl::NULL_ID) {
                            assert(bestAId != nvl::NULL_ID);
                            assert(bestFId != nvl::NULL_ID);

                            const std::vector<Frame>& currentLocalCandidateFrames = localCandidateFrames[cId][bestAId];

                            const Frame& frame1 = currentLocalCandidateFrames[bestFId == 0 ? currentLocalCandidateFrames.size() - 1 : bestFId - 1];
                            const Frame& frame2 = currentLocalCandidateFrames[bestFId];

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
        nvl::animationFrameApplyTransformation(targetFrame, targetLocalBindPose);
        targetAnimation.addKeyframe(targetFrame);




        // ------------------------------------------ FILLING SELECTED KEYFRAMES ------------------------------------------

        //Filling selected keyframes
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Model* currentModel = data.entry(cluster[cId]).model;
            const Skeleton& currentSkeleton = currentModel->skeleton;

            //Local bind pose of the current skeleton
            std::vector<SkeletonTransformation> currentLocalBindPose = nvl::skeletonLocalBindPose(currentSkeleton);

            std::vector<Transformation> clusterTransformations(currentSkeleton.jointNumber());

            //For each joint
            for (JointId jId = 0; jId < currentSkeleton.jointNumber(); ++jId) {
                const Index& animationMode = animationModes[cId];
                const Index& animationId = animationIds[cId];                

                const Index& bestAId = bestKeyframeAnimation[i][cId];
                const Index& bestFId = bestKeyframe[i][cId];

                if (animationMode == BLEND_ANIMATION_FIXED) {
                    if (animationId == BLEND_ANIMATION_ANY) {
                        clusterTransformations[jId] = Transformation::Identity();
                    }
                    else {
                        const std::vector<Frame>& currentLocalSelectedFrames = localFixedFrames[cId];

                        const Frame& fixedFrame1 = currentLocalSelectedFrames[currentFrameId[i][cId] == 0 ? currentLocalSelectedFrames.size() - 1 : currentFrameId[i][cId] - 1];
                        const Frame& fixedFrame2 = currentLocalSelectedFrames[currentFrameId[i][cId]];

                        const Transformation& transformation1 = fixedFrame1.transformation(jId);
                        const Transformation& transformation2 = fixedFrame2.transformation(jId);

                        double fixedTime1 = fixedFrame1.time() + currentTimeOffset[i][cId];
                        double fixedTime2 = fixedFrame2.time() + currentTimeOffset[i][cId];
                        if (currentFrameId[i][cId] == 0) {
                            fixedTime1 = fixedTime2;
                        }

                        double fixedAlpha = fixedTime1 == fixedTime2 ? 1.0 : (currentTime - fixedTime1) / (fixedTime2 - fixedTime1);
                        assert(fixedAlpha >= 0.0 && fixedAlpha <= 1.0);

                        clusterTransformations[jId] = nvl::interpolateAffine(transformation1, transformation2, fixedAlpha);
                    }
                }
                else if (animationMode == BLEND_ANIMATION_KEYFRAME || animationMode == BLEND_ANIMATION_LOOP) {
                    if (bestAId == DUPLICATE_KEYFRAME_TO_BLEND) {
                        assert(animationMode == BLEND_ANIMATION_KEYFRAME);

                        Index prevIndex = i-1;
                        while (bestKeyframeAnimation[prevIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                            prevIndex--;
                        }

                        Index nextIndex = i+1;
                        while (bestKeyframeAnimation[nextIndex][cId] == DUPLICATE_KEYFRAME_TO_BLEND) {
                            nextIndex++;
                        }

                        const Index& prevAId = bestKeyframeAnimation[prevIndex][cId];
                        const Index& prevFId = bestKeyframe[prevIndex][cId];
                        const Index& nextAId = bestKeyframeAnimation[nextIndex][cId];
                        const Index& nextFId = bestKeyframe[nextIndex][cId];

                        assert(prevAId != nvl::NULL_ID);
                        assert(prevFId != nvl::NULL_ID);
                        assert(nextAId != nvl::NULL_ID);
                        assert(nextFId != nvl::NULL_ID);

                        assert(prevAId != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(prevFId != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(nextAId != DUPLICATE_KEYFRAME_TO_BLEND);
                        assert(nextFId != DUPLICATE_KEYFRAME_TO_BLEND);

                        const std::vector<Frame>& prevCandidateFrames = localCandidateFrames[cId][prevAId];
                        const std::vector<Frame>& nextCandidateFrames = localCandidateFrames[cId][nextAId];

                        const Frame& candidateFrame1 = prevCandidateFrames[prevFId];
                        const Frame& candidateFrame2 = nextCandidateFrames[nextFId];
                        double prevTime = times[prevIndex];
                        double nextTime = times[nextIndex];

                        const Transformation& candidateTransformation1 = candidateFrame1.transformation(jId);
                        const Transformation& candidateTransformation2 = candidateFrame2.transformation(jId);

                        double candidateAlpha = (currentTime - prevTime) / (nextTime - prevTime);
                        assert(candidateAlpha >= 0 && candidateAlpha <= 1);

                        clusterTransformations[jId] = nvl::interpolateAffine(candidateTransformation1, candidateTransformation2, candidateAlpha);
                    }
                    else if (bestAId != nvl::NULL_ID) {
                        assert(bestAId != nvl::NULL_ID);
                        assert(bestFId != nvl::NULL_ID);

                        const std::vector<Frame>& currentLocalCandidateFrames = localCandidateFrames[cId][bestAId];

                        const Frame& frame1 = currentLocalCandidateFrames[bestFId == 0 ? currentLocalCandidateFrames.size() - 1 : bestFId - 1];
                        const Frame& frame2 = currentLocalCandidateFrames[bestFId];

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
            nvl::animationFrameApplyTransformation(clusterFrame, currentLocalBindPose);
            clusterAnimations[cId].addKeyframe(clusterFrame);
        }
    }

    if (smoothingIterations > 0) {
        std::cout << "Smoothing keyframes..." << std::endl;
        nvl::animationLaplacianSmoothing(targetAnimation, smoothingIterations, keyframeSmoothAlpha);
    }

    nvl::animationGlobalFromLocal(targetSkeleton, targetAnimation);

#ifndef SIMPLE_ANIMATION_NAME
    targetAnimation.setName("B" + animationName);
#else
    targetAnimation.setName(animationName);
#endif

    Index targetAnimationId = targetModel->addAnimation(targetAnimation);
    resultAnimations.push_back(std::make_pair(entry.id, targetAnimationId));

    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];
        Model* currentModel = data.entry(cluster[cId]).model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        nvl::animationGlobalFromLocal(currentSkeleton, clusterAnimations[cId]);

#ifndef SIMPLE_ANIMATION_NAME
        clusterAnimations[cId].setName("S" + animationName);
#else
        clusterAnimations[cId].setName(animationName);
#endif

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


template<class Frame, class JointId>
void computeWindow(
        const std::vector<Frame>& frames,
        const nvl::Index& fId,
        const unsigned int& windowSize,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence,
        std::vector<nvl::Quaterniond>& quaternionWindow,
        std::vector<nvl::Translation3d>& translationWindow)
{
    typedef typename Frame::Transformation Transformation;
    typedef nvl::Quaterniond Quaternion;
    typedef nvl::Translation3d Translation3d;

    quaternionWindow.resize(windowSize * 2 + 1);
    translationWindow.resize(windowSize * 2 + 1);

    nvl::Index i = 0;
    for (int w = -static_cast<int>(windowSize); w <= +static_cast<int>(windowSize); w++) {
        long long int id = fId + w;

        if (id < 0) {
            id += frames.size();
        }
        else if (id >= static_cast<long long int>(frames.size())) {
            id = id - frames.size();
        }

        const Frame& frame = frames[id];

        Transformation transformation = internal::computeMappedTransformation(frame, mappedJoints, mappedJointConfidence);

        Quaternion quaternion(transformation.rotation());
        Translation3d translation(transformation.translation());

        quaternionWindow[i] = quaternion;
        translationWindow[i] = translation;

        ++i;
    }
    assert(i == windowSize * 2 + 1);
}


inline double windowSimilarity(
        const std::vector<nvl::Quaterniond>& globalQ1,
        const std::vector<nvl::Quaterniond>& globalQ2,
        const std::vector<nvl::Quaterniond>& localQ1,
        const std::vector<nvl::Quaterniond>& localQ2,
        const std::vector<nvl::Translation3d>& globalT1,
        const std::vector<nvl::Translation3d>& globalT2,
        const std::vector<nvl::Translation3d>& localT1,
        const std::vector<nvl::Translation3d>& localT2,
        const std::vector<double>& windowWeights,
        const double& rotationWeight,
        const double& globalWeight,
        const double& localWeight,
        const double& globalDerivativeWeight,
        const double& localDerivativeWeight)
{
    typedef nvl::Index Index;
    typedef nvl::Quaterniond Quaternion;
    typedef nvl::Translation3d Translation3d;

    const double translationWeight = 1.0 - rotationWeight;

    double globalScore = 0.0;
    if (globalWeight > 0.0) {
        assert(globalQ1.size() == globalQ2.size());
        assert(globalQ1.size() == windowWeights.size());

        for (Index i = 0; i < globalQ1.size(); i++) {            
            if (rotationWeight > 0.0) {
                globalScore += rotationWeight * windowWeights[i] * quaternionSimilarity(globalQ1[i], globalQ2[i]);
            }
            if (translationWeight > 0.0) {
                globalScore += translationWeight * windowWeights[i] * translationSimilarity(globalT1[i], globalT2[i]);
            }
        }
    }

    double globalDerivativeScore = 0.0;
    if (globalDerivativeWeight > 0.0 && globalQ1.size() > 1) {
        assert(globalQ1.size() == globalQ2.size());
        assert(globalQ1.size() == windowWeights.size());

        const Index mid = globalQ1.size() / 2;
        for (Index i = 0; i < globalQ1.size(); i++) {
            Quaternion dQ1, dQ2;
            Translation3d dT1, dT2;

            if (i <= mid) {
                if (rotationWeight > 0.0) {
                    dQ1 = globalQ1[i + 1] * globalQ1[i].inverse();
                    dQ2 = globalQ2[i + 1] * globalQ2[i].inverse();
                }

                if (translationWeight > 0.0) {
                    dT1 = globalT1[i + 1] * globalT1[i].inverse();
                    dT2 = globalT2[i + 1] * globalT2[i].inverse();
                }
            }
            else {
                if (rotationWeight > 0.0) {
                    dQ1 = globalQ1[i] * globalQ1[i - 1].inverse();
                    dQ2 = globalQ2[i] * globalQ2[i - 1].inverse();
                }

                if (translationWeight > 0.0) {
                    dT1 = globalT1[i] * globalT1[i - 1].inverse();
                    dT2 = globalT2[i] * globalT2[i - 1].inverse();
                }
            }

            if (rotationWeight > 0.0) {
                globalDerivativeScore += rotationWeight * windowWeights[i] * quaternionSimilarity(dQ1, dQ2);
            }
            if (translationWeight > 0.0) {
                globalDerivativeScore += translationWeight * windowWeights[i] * translationSimilarity(dT1, dT2);
            }
        }
    }

    assert(globalScore >= 0.0 && globalScore <= 1.0);
    assert(globalDerivativeScore >= 0.0 && globalDerivativeScore <= 1.0);





    double localScore = 0.0;
    if (localWeight > 0.0) {
        assert(localQ1.size() == localQ2.size());
        assert(localQ1.size() == windowWeights.size());

        for (Index i = 0; i < localQ1.size(); i++) {
            if (rotationWeight > 0.0) {
                localScore += rotationWeight * windowWeights[i] * quaternionSimilarity(localQ1[i], localQ2[i]);
            }
            if (translationWeight > 0.0) {
                localScore += translationWeight * windowWeights[i] * translationSimilarity(localT1[i], localT2[i]);
            }
        }
    }

    double localDerivativeScore = 0.0;
    if (localDerivativeWeight > 0.0 && localQ1.size() > 1) {
        assert(localQ1.size() == localQ2.size());
        assert(localQ1.size() == windowWeights.size());

        const Index mid = localQ1.size() / 2;
        for (Index i = 0; i < localQ1.size(); i++) {
            Quaternion dQ1, dQ2;
            Translation3d dT1, dT2;

            if (i <= mid) {
                if (rotationWeight > 0.0) {
                    dQ1 = localQ1[i + 1] * localQ1[i].inverse();
                    dQ2 = localQ2[i + 1] * localQ2[i].inverse();
                }

                if (translationWeight > 0.0) {
                    dT1 = localT1[i + 1] * localT1[i].inverse();
                    dT2 = localT2[i + 1] * localT2[i].inverse();
                }
            }
            else {
                if (rotationWeight > 0.0) {
                    dQ1 = localQ1[i] * localQ1[i - 1].inverse();
                    dQ2 = localQ2[i] * localQ2[i - 1].inverse();
                }

                if (translationWeight > 0.0) {
                    dT1 = localT1[i] * localT1[i - 1].inverse();
                    dT2 = localT2[i] * localT2[i - 1].inverse();
                }
            }

            if (rotationWeight > 0.0) {
                localDerivativeScore += rotationWeight * windowWeights[i] * quaternionSimilarity(dQ1, dQ2);
            }
            if (translationWeight > 0.0) {
                localDerivativeScore += translationWeight * windowWeights[i] * translationSimilarity(dT1, dT2);
            }
        }
    }

    assert(localScore >= 0.0 && localScore <= 1.0);
    assert(localDerivativeScore >= 0.0 && localDerivativeScore <= 1.0);

    double score =
            globalWeight * globalScore +
            globalDerivativeWeight * globalDerivativeScore +
            localWeight * localScore +
            localDerivativeWeight * localDerivativeScore;

    assert(score >= 0.0 && score <= 1.0);

    return score;
}

inline double quaternionSimilarity(
        const nvl::Quaterniond& q1,
        const nvl::Quaterniond& q2)
{
    double score = nvl::abs(q1.dot(q2));
    if (nvl::epsEqual(score, 1.0))
        score = 1.0;
    else if (nvl::epsEqual(score, 0.0))
        score = 0.0;
    assert(score >= 0.0 && score <= 1.0);

    return score;
}


inline double translationSimilarity(
        const nvl::Translation3d& t1,
        const nvl::Translation3d& t2)
{
    typedef nvl::Vector3d Vector;

    const double dirWeight = 0.8;
    const double normWeight = 1.0 - dirWeight;

    Vector vec1 = t1 * Vector::Zero();
    Vector vec2 = t2 * Vector::Zero();

    double norm1 = vec1.norm();
    double norm2 = vec2.norm();
    double normScore = (norm1 == norm2 ? 1.0 : (norm1 < norm2 ? norm1 / norm2 : norm2 / norm1));
    assert(normScore >= 0.0 && normScore <= 1.0);

    double dirScore = vec1.normalized().dot(vec2.normalized());
    dirScore = (dirScore + 1) / 2.0;
    assert(dirScore >= 0.0 && dirScore <= 1.0);

    double score = dirWeight * dirScore + normWeight * normScore;
    assert(score >= 0.0 && score <= 1.0);

    return score;
}


template<class S>
inline std::vector<double> computeDistanceWeights(const S& skeleton, const std::vector<typename S::JointId>& seeds)
{    
    typedef typename S::JointId JointId;

    std::vector<double> distanceWeights(skeleton.jointNumber(), 0.0);

    //Joint distance from the merge joints
    std::vector<unsigned int> targetJointDistance = nvl::skeletonJointDistance(skeleton, seeds);

    const unsigned int minDistance = 2;
    const unsigned int maxDistance = 8;

    for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
        const double& distance = targetJointDistance[jId];

        double distanceWeight = 1.0;
        if (distance > minDistance) {
            if (distance > maxDistance) {
                distanceWeight = 0.0;
            }
            else {
                double exponent = -(static_cast<double>(distance) - static_cast<double>(minDistance));
                distanceWeight = nvl::pow(2.0, exponent);
            }
        }

        distanceWeights[jId] = distanceWeight;
    }

    nvl::normalize(distanceWeights);

    return distanceWeights;
}

}
}
