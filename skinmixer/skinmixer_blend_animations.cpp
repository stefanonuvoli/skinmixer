#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_map.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>
#include <nvl/math/numeric_limits.h>

#include <nvl/models/animation_algorithms.h>
#include <iostream>

#define BLEND_ANIMATION_REST nvl::MAX_INDEX - 1
#define BLEND_ANIMATION_KEYFRAME nvl::MAX_INDEX - 2
//#define BLEND_ANIMATION_FIND nvl::MAX_INDEX - 3

namespace skinmixer {

namespace internal {
    template<class T>
    double transformationSimilarityScore(
            const double& time1,
            const T& transformation1,
            const double& time2,
            const T& transformation2,
            const double& candidateTime1,
            const T& candidateTransformation1,
            const double& candidateTime2,
            const T& candidateTransformation2);
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

    for (const nvl::Index& eId : newEntries) {
        Entry& entry = data.entry(eId);

        Model* targetModel = entry.model;
        Skeleton& targetSkeleton = targetModel->skeleton;

        std::vector<Index>& birthEntries = entry.birth.entries;
        std::vector<Index>& animationsIds = entry.blendingAnimations;
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

                        if (jointInfo.confidence == 1.0 && parentJointInfo.confidence == 1.0 && parentCId != cId) {
                            animationWeights[jId][cId] = 0.0;
                        }
                    }
                }
            }

            nvl::normalize(animationWeights[jId]);
        }

        animationsIds.resize(birthEntries.size(), BLEND_ANIMATION_REST);
    }
}

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        nvl::Index& targetAnimationId)
{    
    const double fps = 30;

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
    const std::vector<nvl::Index>& animationIds = entry.blendingAnimations;
    const std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

    Animation targetAnimation;

    std::vector<std::vector<Frame>> fixedFrames(cluster.size());
    std::vector<std::vector<std::vector<Frame>>> candidateFrames(cluster.size());
    std::vector<std::vector<Transformation>> restPoses(cluster.size());

    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        Index animationId = animationIds[cId];

        std::vector<Transformation> currentRestPoses(currentSkeleton.jointNumber());
        for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
            currentRestPoses[jId] = currentSkeleton.joint(jId).restPose();
        }
        nvl::animationComputeLocalTransformations(currentSkeleton, currentRestPoses);
        restPoses[cId] = currentRestPoses;

        if (animationId == BLEND_ANIMATION_REST) {
            continue;
        }
        else if (animationId == BLEND_ANIMATION_KEYFRAME) {
            candidateFrames[cId].resize(currentModel->animationNumber());
            for (nvl::Index aId = 0; aId < currentModel->animationNumber(); ++aId) {
                const Animation& currentAnimation = currentModel->animation(aId);
                for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                    Frame frame = currentAnimation.keyframe(fId);

                    candidateFrames[cId][aId].push_back(frame);
                }
            }
        }
        else {
            const Animation& currentAnimation = currentModel->animation(animationId);
            for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
                Frame frame = currentAnimation.keyframe(fId);

                fixedFrames[cId].push_back(frame);
            }
        }

        nvl::animationBlendFrameTransformations(currentSkeleton, fixedFrames[cId], fps);
        nvl::animationComputeLocalFrames(currentSkeleton, fixedFrames[cId]);
        for (Index fId = 0; fId < fixedFrames[cId].size(); ++fId) {
            times.push_back(fixedFrames[cId][fId].time());
        }

        for (int aId = 0; aId < candidateFrames[cId].size(); aId++) {
            nvl::animationBlendFrameTransformations(currentSkeleton, candidateFrames[cId][aId], fps);
            nvl::animationComputeLocalFrames(currentSkeleton, candidateFrames[cId][aId]);
        }
    }

    std::vector<Transformation> targetLocalRestPoses(targetSkeleton.jointNumber());
    for (JointId jId = 0; jId < targetSkeleton.jointNumber(); jId++) {
        targetLocalRestPoses[jId] = targetSkeleton.joint(jId).restPose();
    }
    nvl::animationComputeLocalTransformations(targetSkeleton, targetLocalRestPoses);

    if (times.empty())
        times.push_back(0.0);

    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());

    std::vector<Index> currentFrameId(cluster.size(), 0);
    std::vector<double> currentTimeOffset(cluster.size(), 0.0);

    //For each time entry
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::vector<Transformation> blendedTransformations(targetSkeleton.jointNumber());

        //Update corresponding frame for the current time
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const std::vector<Frame>& currentSelectedLocalFrames = fixedFrames[cId];
            while (currentFrameId[cId] < currentSelectedLocalFrames.size() && currentSelectedLocalFrames[currentFrameId[cId]].time() + currentTimeOffset[cId] < currentTime) {
                ++currentFrameId[cId];

                if (currentFrameId[cId] >= currentSelectedLocalFrames.size()) {
                    currentTimeOffset[cId] += currentSelectedLocalFrames[currentSelectedLocalFrames.size() - 1].time();
                    currentFrameId[cId] = 0;
                }
            }
        }

#ifndef KEYFRAME_PER_JOINT

        std::vector<double> bestKeyframeScore(cluster.size(), nvl::minLimitValue<double>());
        std::vector<Index> bestAnimation(cluster.size(), nvl::MAX_INDEX);
        std::vector<Index> bestFrame(cluster.size(), nvl::MAX_INDEX);

        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& animationId = animationIds[cId];
            const Model* currentModel = data.entry(cluster[cId]).model;

            if (animationId == BLEND_ANIMATION_KEYFRAME) {
                for (nvl::Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                    const std::vector<Frame>& currentCandidateLocalFrames = candidateFrames[cId][candidateAId];
                    for (Index candidateFId = 0; candidateFId < currentCandidateLocalFrames.size(); candidateFId++) {
                        double keyframeScore = 0.0;
                        int numKeyframeScores = 0;

                        //For each joint
                        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
                            if (!nvl::epsEqual(animationWeights[jId][cId], 0.0)) {
                                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                                const Frame& candidateFrame1 = currentCandidateLocalFrames[candidateFId];
                                const Frame& candidateFrame2 = currentCandidateLocalFrames[(candidateFId + 1) % currentCandidateLocalFrames.size()];

                                for (JointInfo jointInfo : jointInfos) {
                                    double candidateTime1 = candidateFrame1.time();
                                    double candidateTime2 = candidateFrame2.time();
                                    const Transformation& candidateTransformation1 = candidateFrame1.transformation(jointInfo.jId) * restPoses[cId][jointInfo.jId].inverse();
                                    const Transformation& candidateTransformation2 = candidateFrame2.transformation(jointInfo.jId) * restPoses[cId][jointInfo.jId].inverse();

                                    for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                        Index otherAnimationId = animationIds[otherCId];
                                        if (otherAnimationId != BLEND_ANIMATION_KEYFRAME && otherAnimationId != BLEND_ANIMATION_REST) {
                                            const std::vector<Frame>& currentSelectedLocalFrames = fixedFrames[otherCId];

                                            for (Index otherFId = 0; otherFId < currentSelectedLocalFrames.size(); otherFId++) {
                                                const Frame& frame1 = currentSelectedLocalFrames[currentFrameId[otherCId]];
                                                const Frame& frame2 = currentSelectedLocalFrames[(currentFrameId[otherCId] + 1) % currentSelectedLocalFrames.size()];

                                                double time1 = frame1.time();
                                                double time2 = frame2.time();
                                                const Transformation& transformation1 = frame1.transformation(jointInfo.jId) * restPoses[otherCId][jointInfo.jId].inverse();
                                                const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[otherCId][jointInfo.jId].inverse();

                                                double similarity = internal::transformationSimilarityScore(time1, transformation1, time2, transformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                                keyframeScore += similarity * animationWeights[jId][cId] * jointInfo.confidence;
                                                numKeyframeScores++;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        if (numKeyframeScores > 0) {
                            keyframeScore /= numKeyframeScores;
                            if (keyframeScore > bestKeyframeScore[cId]) {
                                bestKeyframeScore[cId] = keyframeScore;
                                bestAnimation[cId] = candidateAId;
                                bestFrame[cId] = candidateFId;
                            }
                        }
                    }

                }
            }
        }

        //For each joint
        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size(), Transformation::Identity());
            std::vector<double> weights(cluster.size(), 0.0);

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

                const Index& clusterId = clusterMap[jointInfo.eId];
                const Index& animationId = animationIds[clusterId];

                //Avoid numerical errors
                if (!nvl::epsEqual(animationWeights[jId][clusterId], 0.0)) {
                    weights[clusterId] = animationWeights[jId][clusterId];

                    if (animationId == BLEND_ANIMATION_REST) {
                        transformations[clusterId] = Transformation::Identity();
                    }
                    else if (animationId == BLEND_ANIMATION_KEYFRAME) {
                        if (bestAnimation[clusterId] != nvl::MAX_INDEX) {
                            const std::vector<Frame>& currentCandidateLocalFrames = candidateFrames[clusterId][bestAnimation[clusterId]];

                            const Frame& frame1 = currentCandidateLocalFrames[bestFrame[clusterId]];
                            const Frame& frame2 = currentCandidateLocalFrames[(bestFrame[clusterId] + 1) % currentCandidateLocalFrames.size()];

                            const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();

                            transformations[clusterId] = transformation2;

                            std::cout << bestAnimation[clusterId] << " " << bestFrame[clusterId] << std::endl;
                        }
                        else {
                            transformations[clusterId] = Transformation::Identity();
                        }
                    }
//                    else if (animationId != BLEND_ANIMATION_FIND) {

//                    }
                    else {
                        const std::vector<Frame>& currentSelectedLocalFrames = fixedFrames[clusterId];

                        const Frame& frame1 = currentSelectedLocalFrames[currentFrameId[clusterId]];
                        const Frame& frame2 = currentSelectedLocalFrames[(currentFrameId[clusterId] + 1) % currentSelectedLocalFrames.size()];

                        double time1 = frame1.time() + currentTimeOffset[clusterId];
                        double time2 = frame2.time() + currentTimeOffset[clusterId];
                        const Transformation& transformation1 = frame1.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();
                        const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();

                        double alpha = (currentTime - time1) / (time2 - time1);

                        transformations[clusterId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
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

            Transformation interpolated = nvl::interpolateAffine(transformations, weights);
            Transformation rot(interpolated.rotation() * targetLocalRestPoses[jId].rotation());
            Transformation tra(nvl::Translation3d(targetLocalRestPoses[jId].translation()));

            if (targetSkeleton.isRoot(jId)) {
                tra = nvl::Translation3d(interpolated.translation()) * tra;
            }

            blendedTransformations[jId] = Transformation(tra * rot);
        }
#else
        //For each joint
        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size(), Transformation::Identity());
            std::vector<double> weights(cluster.size(), 0.0);

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

                const Index& clusterId = clusterMap[jointInfo.eId];
                const Index& animationId = animationIds[clusterId];

                //Avoid numerical errors
                if (!nvl::epsEqual(animationWeights[jId][clusterId], 0.0)) {
                    weights[clusterId] = animationWeights[jId][clusterId];

                    const Model* currentModel = data.entry(jointInfo.eId).model;

                    if (animationId == BLEND_ANIMATION_REST) {
                        transformations[clusterId] = Transformation::Identity();
                    }
                    else if (animationId == BLEND_ANIMATION_KEYFRAME) {
                        double bestKeyframeScore = nvl::minLimitValue<double>();
                        Index bestAnimation = nvl::MAX_INDEX;
                        Index bestFrame = nvl::MAX_INDEX;

                        for (nvl::Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
                            const std::vector<Frame>& currentCandidateLocalFrames = candidateFrames[clusterId][candidateAId];
                            for (Index candidateFId = 0; candidateFId < currentCandidateLocalFrames.size(); candidateFId++) {
                                double keyframeScore = 0.0;
                                int numKeyframeScores = 0;

                                const Frame& candidateFrame1 = currentCandidateLocalFrames[candidateFId];
                                const Frame& candidateFrame2 = currentCandidateLocalFrames[(candidateFId + 1) % currentCandidateLocalFrames.size()];

                                double candidateTime1 = candidateFrame1.time();
                                double candidateTime2 = candidateFrame2.time();
                                const Transformation& candidateTransformation1 = candidateFrame1.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();
                                const Transformation& candidateTransformation2 = candidateFrame2.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();

                                for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                    Index otherAnimationId = animationIds[otherCId];
                                    if (otherAnimationId != BLEND_ANIMATION_KEYFRAME && otherAnimationId != BLEND_ANIMATION_REST) {
                                        const std::vector<Frame>& currentSelectedLocalFrames = fixedFrames[otherCId];

                                        for (Index otherFId = 0; otherFId < currentSelectedLocalFrames.size(); otherFId++) {
                                            const Frame& frame1 = currentSelectedLocalFrames[currentFrameId[otherCId]];
                                            const Frame& frame2 = currentSelectedLocalFrames[(currentFrameId[otherCId] + 1) % currentSelectedLocalFrames.size()];

                                            double time1 = frame1.time();
                                            double time2 = frame2.time();
                                            const Transformation& transformation1 = frame1.transformation(jointInfo.jId) * restPoses[otherCId][jointInfo.jId].inverse();
                                            const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[otherCId][jointInfo.jId].inverse();

                                            double similarity = internal::transformationSimilarityScore(time1, transformation1, time2, transformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                            keyframeScore += similarity;
                                            numKeyframeScores++;
                                        }
                                    }
                                }

                                if (numKeyframeScores > 0) {
                                    keyframeScore /= numKeyframeScores;
                                    if (keyframeScore > bestKeyframeScore) {
                                        bestKeyframeScore = keyframeScore;
                                        bestAnimation = candidateAId;
                                        bestFrame = candidateFId;
                                    }
                                }
                            }
                        }

                        if (bestAnimation != nvl::MAX_INDEX) {
                            const std::vector<Frame>& currentCandidateLocalFrames = candidateFrames[clusterId][bestAnimation];

                            const Frame& frame1 = currentCandidateLocalFrames[bestFrame];
                            const Frame& frame2 = currentCandidateLocalFrames[(bestFrame + 1) % currentCandidateLocalFrames.size()];

                            const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();

                            transformations[clusterId] = transformation2;
                        }
                        else {
                            transformations[clusterId] = Transformation::Identity();
                        }
                    }
//                    else if (animationId != BLEND_ANIMATION_FIND) {

//                    }
                    else {
                        const std::vector<Frame>& currentSelectedLocalFrames = fixedFrames[clusterId];

                        const Frame& frame1 = currentSelectedLocalFrames[currentFrameId[clusterId]];
                        const Frame& frame2 = currentSelectedLocalFrames[(currentFrameId[clusterId] + 1) % currentSelectedLocalFrames.size()];

                        double time1 = frame1.time() + currentTimeOffset[clusterId];
                        double time2 = frame2.time() + currentTimeOffset[clusterId];
                        const Transformation& transformation1 = frame1.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();
                        const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * restPoses[clusterId][jointInfo.jId].inverse();

                        double alpha = (currentTime - time1) / (time2 - time1);

                        transformations[clusterId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
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

            Transformation interpolated = nvl::interpolateAffine(transformations, weights);
            Transformation rot(interpolated.rotation() * targetLocalRestPoses[jId].rotation());
            Transformation tra(nvl::Translation3d(targetLocalRestPoses[jId].translation()));

            if (targetSkeleton.isRoot(jId)) {
                tra = nvl::Translation3d(interpolated.translation()) * tra;
            }

            blendedTransformations[jId] = Transformation(tra * rot);
        }
#endif

        Frame newFrame(currentTime, blendedTransformations);
        nvl::animationComputeGlobalFrame(targetSkeleton, newFrame);
        targetAnimation.addKeyframe(newFrame);
    }

    targetAnimation.setName("Blended");
    if (targetAnimationId == nvl::MAX_INDEX) {
        targetAnimationId = targetModel->addAnimation(targetAnimation);
    }
    else {
        targetModel->setAnimation(targetAnimationId, targetAnimation);
    }
}


//template<class Model>
//nvl::Index findBestAnimation(
//        SkinMixerData<Model>& data,
//        typename SkinMixerData<Model>::Entry& entry,
//        const nvl::Index& index)
//{
//    typedef typename nvl::Index Index;

//    typedef typename SkinMixerData<Model>::Entry Entry;
//    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

//    typedef typename Model::Skeleton Skeleton;
//    typedef typename Skeleton::JointId JointId;

//    typedef typename Model::Animation Animation;
//    typedef typename Animation::Frame Frame;
//    typedef typename Animation::Transformation Transformation;

//    Model* targetModel = entry.model;
//    Skeleton& targetSkeleton = targetModel->skeleton;
//    std::vector<Index>& cluster = entry.birth.entries;
//    Model* findModel = data.entry(cluster[index]).model;
//    const std::vector<nvl::Index>& animationIds = entry.blendingAnimations;

//    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

//    std::vector<std::vector<Frame>> existingLocalFrames(cluster.size());
//    std::vector<std::vector<Frame>> candidateLocalFrames(findModel->animationNumber());
//    std::vector<std::vector<Transformation>> existingLocalRestPose(cluster.size());
//    std::vector<std::vector<Transformation>> candidateLocalRestPose(findModel->animationNumber());
//    std::vector<double> times;

//    for (Index cId = 0; cId < cluster.size(); ++cId) {
//        Index eId = cluster[cId];
//        const Entry& currentEntry = data.entry(eId);
//        const Model* currentModel = currentEntry.model;
//        const Skeleton& currentSkeleton = currentModel->skeleton;

//        Index animationId = animationIds[cId];
//        if (cId == index) {
//            assert(animationId == BLEND_ANIMATION_FIND);

//            for (nvl::Index aId = 0; aId < findModel->animationNumber(); ++aId) {
//                const Animation& currentAnimation = currentModel->animation(aId);
//                for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
//                    Frame frame = currentAnimation.keyframe(fId);

//                    nvl::animationComputeLocalFrame(currentSkeleton, frame);
//                    candidateLocalFrames[aId].push_back(frame);

//                    std::vector<Transformation> restPoses(currentSkeleton.jointNumber());
//                    for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
//                        restPoses[jId] = currentSkeleton.joint(jId).restPose();
//                    }
//                    nvl::animationComputeLocalTransformations(currentSkeleton, restPoses);
//                    candidateLocalRestPose[aId] = restPoses;

//                    times.push_back(frame.time());
//                }
//            }
//        }
//        else {
//            std::vector<Transformation> restPoses(currentSkeleton.jointNumber());
//            for (JointId jId = 0; jId < currentSkeleton.jointNumber(); jId++) {
//                restPoses[jId] = currentSkeleton.joint(jId).restPose();
//            }
//            nvl::animationComputeLocalTransformations(currentSkeleton, restPoses);
//            existingLocalRestPose[cId] = restPoses;

//            if (animationId == BLEND_ANIMATION_REST || animationId == BLEND_ANIMATION_KEYFRAME) {
//                continue;
//            }

//            assert(animationId != BLEND_ANIMATION_FIND);

//            const Animation& currentAnimation = currentModel->animation(animationId);
//            for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
//                Frame frame = currentAnimation.keyframe(fId);

//                nvl::animationComputeLocalFrame(currentSkeleton, frame);
//                existingLocalFrames[cId].push_back(frame);

//                times.push_back(frame.time());
//            }
//        }
//    }

//    if (times.empty())
//        times.push_back(0.0);

//    std::sort(times.begin(), times.end());
//    times.erase(std::unique(times.begin(), times.end()), times.end());

//    double bestScore = nvl::minLimitValue<double>();
//    nvl::Index bestAnimation = nvl::MAX_INDEX;

//    for (nvl::Index aId = 0; aId < findModel->animationNumber(); ++aId) {
//        double score = 0.0;
//        Index numScores = 0;

//        if (candidateLocalFrames[aId].empty())
//            continue;

//        std::vector<Index> currentFrameId(cluster.size(), 0);
//        Index candidateFrameId = 0;
//        for (Index i = 0; i < times.size(); ++i) {
//            const double& currentTime = times[i];

//            for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
//                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

//                double candidateTime1;
//                double candidateTime2;
//                Transformation candidateTransformation1;
//                Transformation candidateTransformation2;

//                for (JointInfo jointInfo : jointInfos) {
//                    assert(jointInfo.jId != nvl::MAX_INDEX);
//                    assert(jointInfo.eId != nvl::MAX_INDEX);
//                    assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

//                    const Index& clusterId = clusterMap[jointInfo.eId];

//                    if (clusterId == index) {
//                        assert(animationId == BLEND_ANIMATION_FIND);

//                        const std::vector<Frame>& currentCandidateLocalFrames = candidateLocalFrames[aId];
//                        if (candidateFrameId < currentCandidateLocalFrames.size() - 1 && currentCandidateLocalFrames[candidateFrameId].time() < currentTime) {
//                            ++candidateFrameId;
//                            assert(currentCandidateLocalFrames[candidateFrameId].time() >= currentTime);
//                        }

//                        if (candidateFrameId < currentCandidateLocalFrames.size() - 1) {
//                            const Frame& frame1 = currentCandidateLocalFrames[candidateFrameId];
//                            const Frame& frame2 = currentCandidateLocalFrames[candidateFrameId + 1];

//                            candidateTime1 = frame1.time();
//                            candidateTime2 = frame2.time();
//                            candidateTransformation1 = frame1.transformation(jointInfo.jId) * candidateLocalRestPose[aId][jointInfo.jId].inverse();
//                            candidateTransformation2 = frame2.transformation(jointInfo.jId) * candidateLocalRestPose[aId][jointInfo.jId].inverse();
//                        }
//                        else {
//                            candidateTime1 = currentTime;
//                            candidateTime2 = currentTime;
//                            candidateTransformation1 = currentCandidateLocalFrames[currentCandidateLocalFrames.size() - 1].transformation(jointInfo.jId) * candidateLocalRestPose[aId][jointInfo.jId].inverse();
//                            candidateTransformation2 = currentCandidateLocalFrames[currentCandidateLocalFrames.size() - 1].transformation(jointInfo.jId) * candidateLocalRestPose[aId][jointInfo.jId].inverse();
//                        }
//                    }
//                }

//                for (JointInfo jointInfo : jointInfos) {
//                    assert(jointInfo.jId != nvl::MAX_INDEX);
//                    assert(jointInfo.eId != nvl::MAX_INDEX);
//                    assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

//                    const Index& clusterId = clusterMap[jointInfo.eId];
//                    const Index& animationId = animationIds[clusterId];

//                    if (animationId == BLEND_ANIMATION_REST || animationId == BLEND_ANIMATION_KEYFRAME || existingLocalFrames[clusterId].empty()) {
//                        continue;
//                    }

//                    double time1;
//                    double time2;
//                    Transformation transformation1;
//                    Transformation transformation2;

//                    const std::vector<Frame>& currentExistingLocalFrames = existingLocalFrames[clusterId];

//                    if (currentFrameId[clusterId] < currentExistingLocalFrames.size() - 1 && currentExistingLocalFrames[currentFrameId[clusterId]].time() < currentTime) {
//                        ++currentFrameId[clusterId];
//                        assert(currentExistingLocalFrames[currentFrameId[cId]].time() >= currentTime);
//                    }

//                    if (currentFrameId[clusterId] < currentExistingLocalFrames.size() - 1) {
//                        const Frame& frame1 = currentExistingLocalFrames[currentFrameId[clusterId]];
//                        const Frame& frame2 = currentExistingLocalFrames[currentFrameId[clusterId] + 1];

//                        time1 = frame1.time();
//                        time2 = frame2.time();
//                        transformation1 = frame1.transformation(jointInfo.jId) * existingLocalRestPose[clusterId][jointInfo.jId].inverse();
//                        transformation2 = frame2.transformation(jointInfo.jId) * existingLocalRestPose[clusterId][jointInfo.jId].inverse();
//                    }
//                    else {
//                        time1 = currentTime;
//                        time2 = currentTime;
//                        transformation1 = currentExistingLocalFrames[currentExistingLocalFrames.size() - 1].transformation(jointInfo.jId) * existingLocalRestPose[clusterId][jointInfo.jId].inverse();
//                        transformation2 = currentExistingLocalFrames[currentExistingLocalFrames.size() - 1].transformation(jointInfo.jId) * existingLocalRestPose[clusterId][jointInfo.jId].inverse();
//                    }

//                    double similarity = internal::transformationSimilarityScore(time1, transformation1, time2, transformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2, currentTime);
//                    score += similarity * jointInfo.confidence;
//                    numScores++;
//                }
//            }
//        }

//        if (numScores > 0) {
//            score /= numScores;
//            if (score > bestScore) {
//                bestScore = score;
//                bestAnimation = aId;
//            }
//        }
//    }

//    return bestAnimation;
//}

namespace internal {
    template<class T>
    double transformationSimilarityScore(
            const double& targetTime1,
            const T& targetTransformation1,
            const double& targetTime2,
            const T& targetTransformation2,
            const double& candidateTime1,
            const T& candidateTransformation1,
            const double& candidateTime2,
            const T& candidateTransformation2)
    {
        const double velocityWeight = 0.5;
        const double similarityWeight = 1.0 - velocityWeight;
        


        nvl::Quaterniond targetQuaternion(targetTransformation2.rotation());
        nvl::Quaterniond candidateQuaternion(candidateTransformation2.rotation());

        double similarityScore = std::fabs(targetQuaternion.dot(candidateQuaternion));




        nvl::Vector4d candidateVec1(nvl::Quaterniond(candidateTransformation1.rotation()    ).coeffs());
        nvl::Vector4d candidateVec2(nvl::Quaterniond(candidateTransformation2.rotation()).coeffs());

        nvl::Vector4d candidateDifference;

        double candidateDot = candidateVec1.dot(candidateVec2);
        if (candidateDot < 0.0) {
            candidateDifference = candidateVec2 - candidateVec1;
        }
        else {
            candidateDifference = candidateVec2 + candidateVec1;
        }
        candidateDifference /= (candidateTime2 - candidateTime1);

        nvl::Vector4d targetVec1(nvl::Quaterniond(targetTransformation1.rotation()).coeffs());
        nvl::Vector4d targetVec2(nvl::Quaterniond(targetTransformation2.rotation()).coeffs());

        nvl::Vector4d targetDifference;

        double targetDot = targetVec1.dot(targetVec2);
        if (targetDot < 0.0) {
            targetDifference = targetVec2 - targetVec1;
        }
        else {
            targetDifference = targetVec2 + targetVec1;
        }
        targetDifference /= (targetTime2 - targetTime1);

        double velocityScore = std::fabs(targetDifference.dot(candidateDifference));




        return similarityWeight * similarityScore + velocityWeight * velocityScore;
    }
}
}
