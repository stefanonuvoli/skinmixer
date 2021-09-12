#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_map.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>
#include <nvl/math/numeric_limits.h>

#include <nvl/models/animation_algorithms.h>
#include <iostream>

#define BLEND_ANIMATION_REST nvl::MAX_INDEX - 1
#define BLEND_ANIMATION_KEYFRAME nvl::MAX_INDEX - 2
#define KEYFRAME_SELECTION_VERBOSITY
#define SIMILARITY_WITH_INTERPOLATED_TRANSFORMATION

namespace skinmixer {

namespace internal {

template<class Frame, class JointId>
typename Frame::Transformation computeMappedTransformation(
        const Frame& frame,
        const std::vector<JointId>& mappedJoints,
        const std::vector<double>& mappedJointConfidence);

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

                        if (jointInfo.confidence == 1.0 && parentJointInfo.confidence == 1.0 && parentCId == cId) {
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
    const std::vector<nvl::Index>& animationIds = entry.blendingAnimations;
    const std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

    Animation targetAnimation;
    std::vector<Animation> clusterAnimations(cluster.size());

    std::vector<std::vector<Frame>> fixedFrames(cluster.size());
    std::vector<std::vector<std::vector<Frame>>> candidateFrames(cluster.size());

    //Fill candidate and fixed frames
    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        Index animationId = animationIds[cId];

        if (animationId == BLEND_ANIMATION_REST) {
            continue;
        }
        else if (animationId == BLEND_ANIMATION_KEYFRAME) {
            candidateFrames[cId].resize(currentModel->animationNumber());
            for (Index aId = 0; aId < currentModel->animationNumber(); ++aId) {
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

        nvl::animationBlendFrameTransformations(fixedFrames[cId], fps);
        for (Index aId = 0; aId < candidateFrames[cId].size(); aId++) {
            nvl::animationBlendFrameTransformations(candidateFrames[cId][aId], fps);
        }

        for (Index fId = 0; fId < fixedFrames[cId].size(); ++fId) {
            times.push_back(fixedFrames[cId][fId].time());
        }
    }

    //At least one time entry
    if (times.empty())
        times.push_back(0.0);
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());

    //Variables for iteration
    std::vector<Index> currentFrameId(cluster.size(), 0);
    std::vector<double> currentTimeOffset(cluster.size(), 0.0);


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

    //For each time entry
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::vector<Transformation> blendedTransformations(targetSkeleton.jointNumber());

        //Update corresponding frame for the current time
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const std::vector<Frame>& currentFixedFrames = fixedFrames[cId];
            while (currentFrameId[cId] < currentFixedFrames.size() && currentFixedFrames[currentFrameId[cId]].time() + currentTimeOffset[cId] < currentTime) {
                ++currentFrameId[cId];

                if (currentFrameId[cId] >= currentFixedFrames.size()) {
                    currentTimeOffset[cId] += currentFixedFrames[currentFixedFrames.size() - 1].time();
                    currentFrameId[cId] = 0;
                }
            }
        }

        std::vector<unsigned int> targetJointDistance = nvl::skeletonJointDistance(targetSkeleton, entry.birth.mergeJoints);

        //Compute best frames for select best keyframe animation
        std::vector<double> bestFrameScore(cluster.size(), nvl::minLimitValue<double>());
        std::vector<Index> bestAnimation(cluster.size(), nvl::MAX_INDEX);
        std::vector<Index> bestFrame(cluster.size(), nvl::MAX_INDEX);
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& animationId = animationIds[cId];
            const Model* currentModel = data.entry(cluster[cId]).model;

            if (animationId == BLEND_ANIMATION_KEYFRAME) {
                for (nvl::Index candidateAId = 0; candidateAId < currentModel->animationNumber(); ++candidateAId) {
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
                            double distanceWeight = 1.0;
                            if (targetJointDistance[jId] > 2) {
                                if (targetJointDistance[jId] > 8) {
                                    distanceWeight = 0.0;
                                }
                                else {
                                    double exponent = -(static_cast<double>(targetJointDistance[jId]) - 2.0);
                                    distanceWeight = nvl::pow(2.0, exponent);
                                }
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
                                Index otherAnimationId = animationIds[otherCId];
                                if (otherAnimationId != BLEND_ANIMATION_KEYFRAME && otherAnimationId != BLEND_ANIMATION_REST) {
                                    const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                    const Frame& frame1 = currentFixedFrames[currentFrameId[otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[otherCId] - 1];
                                    const Frame& frame2 = currentFixedFrames[currentFrameId[otherCId]];
                                    double time1 = frame1.time();
                                    double time2 = frame2.time();

                                    Transformation transformation1 = internal::computeMappedTransformation(frame1, mappedJoints[otherCId], mappedJointConfidence[otherCId]);
                                    Transformation transformation2 = internal::computeMappedTransformation(frame2, mappedJoints[otherCId], mappedJointConfidence[otherCId]);

                                    double similarity = internal::transformationSimilarityScore(time1, transformation1, time2, transformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
                                    assert(similarity >= 0 && similarity <= 1);

                                    jointFrameScore += similarity;

                                    numOtherAnimations++;
                                }
                            }
#else
                            std::vector<std::vector<double>> normalizedMappedJointConfidence = mappedJointConfidence;
                            for (Index i = 0; i < mappedJoints.size(); ++i) {
                                nvl::normalize(normalizedMappedJointConfidence[i]);
                            }

                            for (Index mi = 0; mi < mappedJoints[cId].size(); ++mi) {
                                const JointId& candidateMappedJoint = mappedJoints[cId][mi];
                                const JointId& candidateMappedJointConfidence = normalizedMappedJointConfidence[cId][mi];

                                const Transformation& candidateTransformation1 = candidateFrame1.transformation(candidateMappedJoint);
                                const Transformation& candidateTransformation2 = candidateFrame2.transformation(candidateMappedJoint);

                                for (Index otherCId = 0; otherCId < cluster.size(); ++otherCId) {
                                    Index otherAnimationId = animationIds[otherCId];
                                    if (otherAnimationId != BLEND_ANIMATION_KEYFRAME && otherAnimationId != BLEND_ANIMATION_REST) {
                                        for (Index mj = 0; mj < mappedJoints[otherCId].size(); ++mj) {
                                            const JointId& currentMappedJoint = mappedJoints[otherCId][mj];
                                            const JointId& currentMappedJointConfidence = normalizedMappedJointConfidence[otherCId][mj];

                                            const std::vector<Frame>& currentFixedFrames = globalFixedFrames[otherCId];

                                            const Frame& frame1 = currentFixedFrames[currentFrameId[otherCId] == 0 ? currentFixedFrames.size() - 1 : currentFrameId[otherCId] - 1];
                                            const Frame& frame2 = currentFixedFrames[currentFrameId[otherCId]];
                                            double time1 = frame1.time();
                                            double time2 = frame2.time();

                                            const Transformation& transformation1 = frame1.transformation(currentMappedJoint);
                                            const Transformation& transformation2 = frame2.transformation(currentMappedJoint);

                                            double similarity = internal::transformationSimilarityScore(time1, transformation1, time2, transformation2, candidateTime1, candidateTransformation1, candidateTime2, candidateTransformation2);
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

                        if (frameScore > bestFrameScore[cId]) {
                            bestFrameScore[cId] = frameScore;
                            bestAnimation[cId] = candidateAId;
                            bestFrame[cId] = candidateFId;
                        }
                    }

                }
            }
        }

#ifdef KEYFRAME_SELECTION_VERBOSITY
        std::cout << ">>>> " << targetAnimation.keyframeNumber() << " - Time: " << currentTime << std::endl;
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& animationId = animationIds[cId];
            if (animationId == BLEND_ANIMATION_KEYFRAME) {
                if (bestAnimation[cId] != nvl::MAX_INDEX) {
                    std::cout << cId << " -> Animation: " << bestAnimation[cId] << " - Frame: " << bestFrame[cId] << std::endl;
                }
            }
        }
#endif

        //For each joint
        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size(), Transformation::Identity());
            std::vector<double> weights(cluster.size(), 0.0);

//            for (Index cId = 0; cId < cluster.size(); ++cId) {
//                const Index& animationId = animationIds[cId];

//                JointId correspondingJoint = nvl::MAX_INDEX;
//                double bestConfidence = nvl::minLimitValue<double>();
//                for (JointInfo jointInfo : jointInfos) {
//                    if (clusterMap[jointInfo.eId] == cId) {
//                        if (jointInfo.confidence > bestConfidence) {
//                            correspondingJoint = jointInfo.jId;
//                            bestConfidence = jointInfo.confidence;
//                        }
//                    }
//                }

//                //Avoid numerical errors
//                if (!nvl::epsEqual(animationWeights[jId][cId], 0.0)) {
//                    assert(correspondingJoint != nvl::MAX_INDEX);
//                    weights[cId] = animationWeights[jId][cId];

//                    if (animationId == BLEND_ANIMATION_REST) {
//                        transformations[cId] = Transformation::Identity();
//                    }
//                    else if (animationId == BLEND_ANIMATION_KEYFRAME) {
//                        if (bestAnimation[cId] != nvl::MAX_INDEX) {
//                            const std::vector<Frame>& currentCandidateFrames = candidateFrames[cId][bestAnimation[cId]];

//                            const Frame& frame1 = currentCandidateFrames[bestFrame[cId] == 0 ? currentCandidateFrames.size() - 1 : bestFrame[cId] - 1];
//                            const Frame& frame2 = currentCandidateFrames[bestFrame[cId]];

//                            const Transformation& transformation2 = frame2.transformation(correspondingJoint);

//                            transformations[cId] = transformation2;
//                        }
//                        else {
//                            transformations[cId] = Transformation::Identity();
//                        }
//                    }
//                    else {
//                        const std::vector<Frame>& currentSelectedFrames = fixedFrames[cId];

//                        const Frame& frame1 = currentSelectedFrames[currentFrameId[cId] == 0 ? currentSelectedFrames.size() - 1 : currentFrameId[cId] - 1];
//                        const Frame& frame2 = currentSelectedFrames[currentFrameId[cId]];

//                        double time1 = frame1.time() + currentTimeOffset[cId];
//                        double time2 = frame2.time() + currentTimeOffset[cId];
//                        const Transformation& transformation1 = frame1.transformation(correspondingJoint);
//                        const Transformation& transformation2 = frame2.transformation(correspondingJoint);

//                        double alpha = time1 == time2 ? 0 : (currentTime - time1) / (time2 - time1);
//                        assert(alpha >= 0.0 && alpha <= 1.0);

//                        transformations[cId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
//                    }
//                }
//            }

            for (Index cId = 0; cId < cluster.size(); ++cId) {
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

                    if (animationId == BLEND_ANIMATION_REST) {
                        transformations[cId] = Transformation::Identity();
                    }
                    else if (animationId == BLEND_ANIMATION_KEYFRAME) {
                        if (bestAnimation[cId] != nvl::MAX_INDEX) {
                            const std::vector<Frame>& currentCandidateFrames = candidateFrames[cId][bestAnimation[cId]];

                            const Frame& frame1 = currentCandidateFrames[bestFrame[cId] == 0 ? currentCandidateFrames.size() - 1 : bestFrame[cId] - 1];
                            const Frame& frame2 = currentCandidateFrames[bestFrame[cId]];

                            Transformation transformation2 = internal::computeMappedTransformation(frame2, mappedJoints[cId], mappedJointConfidence[cId]);

                            transformations[cId] = transformation2;
                        }
                        else {
                            transformations[cId] = Transformation::Identity();
                        }
                    }
                    else {
                        const std::vector<Frame>& currentSelectedFrames = fixedFrames[cId];

                        const Frame& frame1 = currentSelectedFrames[currentFrameId[cId] == 0 ? currentSelectedFrames.size() - 1 : currentFrameId[cId] - 1];
                        const Frame& frame2 = currentSelectedFrames[currentFrameId[cId]];

                        double time1 = frame1.time() + currentTimeOffset[cId];
                        double time2 = frame2.time() + currentTimeOffset[cId];

                        Transformation transformation1 = internal::computeMappedTransformation(frame1, mappedJoints[cId], mappedJointConfidence[cId]);
                        Transformation transformation2 = internal::computeMappedTransformation(frame2, mappedJoints[cId], mappedJointConfidence[cId]);


                        double alpha = time1 == time2 ? 0 : (currentTime - time1) / (time2 - time1);
                        assert(alpha >= 0.0 && alpha <= 1.0);

                        transformations[cId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
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
            blendedTransformations[jId] = interpolated;
        }

        Frame targetFrame(currentTime, blendedTransformations);
        targetAnimation.addKeyframe(targetFrame);



        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Model* currentModel = data.entry(cluster[cId]).model;
            const Skeleton& currentSkeleton = currentModel->skeleton;

            std::vector<Transformation> clusterTransformations(currentSkeleton.jointNumber());

            //For each joint
            for (JointId jId = 0; jId < currentSkeleton.jointNumber(); ++jId) {
                const Index& animationId = animationIds[cId];

                if (animationId == BLEND_ANIMATION_REST) {
                    clusterTransformations[jId] = Transformation::Identity();
                }
                else if (animationId == BLEND_ANIMATION_KEYFRAME) {
                    if (bestAnimation[cId] != nvl::MAX_INDEX) {
                        const std::vector<Frame>& currentCandidateFrames = candidateFrames[cId][bestAnimation[cId]];

                        const Frame& frame1 = currentCandidateFrames[bestFrame[cId] == 0 ? currentCandidateFrames.size() - 1 : bestFrame[cId] - 1];
                        const Frame& frame2 = currentCandidateFrames[bestFrame[cId]];

                        const Transformation& transformation2 = frame2.transformation(jId);

                        clusterTransformations[jId] = transformation2;
                    }
                    else {
                        clusterTransformations[jId] = Transformation::Identity();
                    }
                }
                else {
                    const std::vector<Frame>& currentSelectedFrames = fixedFrames[cId];

                    const Frame& frame1 = currentSelectedFrames[currentFrameId[cId] == 0 ? currentSelectedFrames.size() - 1 : currentFrameId[cId] - 1];
                    const Frame& frame2 = currentSelectedFrames[currentFrameId[cId]];

                    double time1 = frame1.time() + currentTimeOffset[cId];
                    double time2 = frame2.time() + currentTimeOffset[cId];
                    const Transformation& transformation1 = frame1.transformation(jId);
                    const Transformation& transformation2 = frame2.transformation(jId);

                    double alpha = time1 == time2 ? 0 : (currentTime - time1) / (time2 - time1);
                    assert(alpha >= 0.0 && alpha <= 1.0);

                    clusterTransformations[jId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
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
        const double& targetTime1,
        const T& targetTransformation1,
        const double& targetTime2,
        const T& targetTransformation2,
        const double& candidateTime1,
        const T& candidateTransformation1,
        const double& candidateTime2,
        const T& candidateTransformation2)
{
    const double globalWeight = 0.8;
    const double localWeight = 0.2;


    nvl::Quaterniond targetQuaternion1(targetTransformation1.rotation());
    nvl::Quaterniond candidateQuaternion1(candidateTransformation1.rotation());
    nvl::Quaterniond targetQuaternion2(targetTransformation2.rotation());
    nvl::Quaterniond candidateQuaternion2(candidateTransformation2.rotation());
    targetQuaternion1.normalize();
    candidateQuaternion1.normalize();
    targetQuaternion2.normalize();
    candidateQuaternion2.normalize();




    double globalScore = nvl::abs(targetQuaternion2.dot(candidateQuaternion2));
    if (nvl::epsEqual(globalScore, 1.0))
        globalScore = 1.0;
    else if (nvl::epsEqual(globalScore, 0.0))
        globalScore = 0.0;
    assert(globalScore >= 0.0 && globalScore <= 1.0);




    nvl::Quaterniond localTargetQuaternion(targetQuaternion2 * targetQuaternion1.inverse());
    nvl::Quaterniond localCandidateQuaternion(candidateQuaternion2 * candidateQuaternion1.inverse());
    localTargetQuaternion.normalize();
    localCandidateQuaternion.normalize();

    double localScore = nvl::abs(localTargetQuaternion.dot(localCandidateQuaternion));
    if (nvl::epsEqual(localScore, 1.0))
        localScore = 1.0;
    else if (nvl::epsEqual(localScore, 0.0))
        localScore = 0.0;
    assert(localScore >= 0.0 && localScore <= 1.0);




    return globalWeight * globalScore + localWeight * localScore;
}

}
}
