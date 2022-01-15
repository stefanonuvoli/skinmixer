#include "skinmixer_data.h"

#include <nvl/math/interpolation.h>
#include <nvl/math/normalization.h>

#include <nvl/models/algorithms/animation_skinning.h>
#include <nvl/models/algorithms/skeleton_adjacencies.h>
#include <nvl/models/algorithms/model_deformation.h>
#include <nvl/models/algorithms/model_clean.h>

#include <unordered_set>

#define MIN_SKINNING_WEIGHT 0.05

namespace skinmixer {

template<class Model>
SkinMixerData<Model>::SkinMixerData()
{

}

template<class Model>
SkinMixerData<Model>::~SkinMixerData()
{
    this->clear();
}

template<class Model>
nvl::Index SkinMixerData<Model>::addEntry(Model* model)
{
    Entry entry;

    entry.id = vEntries.size();

    entry.model = model;

    entry.lastOriginalAnimationId = model->animationNumber();

    vEntries.push_back(entry);
    vModelMap.insert(std::make_pair(model, entry.id));

    return entry.id;
}

template<class Model>
void SkinMixerData<Model>::deleteEntry(Model* model)
{
    typename std::unordered_map<Model*, Index>::iterator it = vModelMap.find(model);
    if (it != vModelMap.end()) {
        it->second->clear();
        vModelMap.erase(model);
    }
}

template<class Model>
nvl::Size SkinMixerData<Model>::entryNumber() const
{
    return vEntries.size();
}

template<class Model>
const std::vector<typename SkinMixerData<Model>::Entry>& SkinMixerData<Model>::entries() const
{
    return vEntries;
}

template<class Model>
std::vector<typename SkinMixerData<Model>::Entry>& SkinMixerData<Model>::entries()
{
    return vEntries;
}

template<class Model>
typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(const Index& id)
{
    return vEntries.at(id);
}

template<class Model>
const typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(const Index& id) const
{
    return vEntries.at(id);
}

template<class Model>
typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entryFromModel(const Model* model)
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
const typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entryFromModel(const Model* model) const
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
nvl::Index SkinMixerData<Model>::addAction(const Action& action)
{
    vActions.push_back(action);
    return vActions.size() - 1;
}

template<class Model>
void SkinMixerData<Model>::removeAction(const SkinMixerData::Index& index)
{
    vActions.erase(vActions.begin() + index);
}

template<class Model>
nvl::Size SkinMixerData<Model>::actionNumber() const
{
    return vActions.size();
}

template<class Model>
const std::vector<typename SkinMixerData<Model>::Action>& SkinMixerData<Model>::actions() const
{
    return vActions;
}

template<class Model>
typename SkinMixerData<Model>::Action& SkinMixerData<Model>::action(const Index& id)
{
    return vActions.at(id);
}

template<class Model>
const typename SkinMixerData<Model>::Action& SkinMixerData<Model>::action(const Index& id) const
{
    return vActions.at(id);
}

template<class Model>
void SkinMixerData<Model>::applyJointDeformation(const Index& entryId, const JointId& jId, const Rotation& rotation, const Translation& translation)
{
    Entry& entry = this->entry(entryId);
    return this->applyJointDeformation(entry, jId, rotation, translation);
}

template<class Model>
void SkinMixerData<Model>::applyJointDeformation(Entry& entry, const JointId& jId, const Rotation& rotation, const Translation& translation)
{
    const Model* model = entry.model;

    std::vector<Affine>& rotations = entry.rotations;
    std::vector<Translation>& translations = entry.translations;

    if (rotations.empty() || translations.empty()) {
        rotations.resize(model->skeleton.jointNumber(), Affine::Identity());
        translations.resize(model->skeleton.jointNumber(), Translation::Identity());
    }

    rotations[jId] = rotation * rotations[jId];
    translations[jId] = translation * translations[jId];
}

template<class Model>
void SkinMixerData<Model>::resetJointDeformation(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->computeActionDeformation(entry);
}

template<class Model>
void SkinMixerData<Model>::resetJointDeformation(Entry& entry)
{
    entry.rotations.clear();
    entry.translations.clear();
}

template<class Model>
std::vector<typename SkinMixerData<Model>::DualQuaternion> SkinMixerData<Model>::computeJointDeformation(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->computeJointDeformation(entry);
}

template<class Model>
std::vector<typename SkinMixerData<Model>::DualQuaternion> SkinMixerData<Model>::computeJointDeformation(Entry& entry)
{
    const std::vector<Affine>& rotations = entry.rotations;
    const std::vector<Translation>& translations = entry.translations;

    Model* model = entry.model;

    std::vector<DualQuaternion> deformations(entry.model->skeleton.jointNumber(), DualQuaternion::Identity());

    bool deformationFound = false;

    if (!rotations.empty() && !translations.empty()) {
        //Generate dual quaternions
        std::vector<Affine> propagatedRotations(entry.model->skeleton.jointNumber(), Affine::Identity());
        std::vector<Translation> propagatedTranslations(entry.model->skeleton.jointNumber(), Translation::Identity());

        for (Index jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
            propagatedRotations[jId] = model->skeleton.jointBindPose(jId) * rotations[jId] * model->skeleton.jointBindPose(jId).inverse();
            propagatedTranslations[jId] = translations[jId];
        }

        //Propagate
        nvl::skeletonPoseGlobalFromLocal(model->skeleton, propagatedRotations);
        nvl::skeletonPoseGlobalFromLocal(model->skeleton, propagatedTranslations);

        //Set deformations
        for (Index jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
            Affine transformation = propagatedTranslations[jId] * propagatedRotations[jId];

            if (!transformation.isApprox(Affine::Identity())) {
                deformations[jId] = DualQuaternion(transformation);
                deformationFound = true;
            }
        }
    }

    if (!deformationFound) {
        deformations.clear();
    }

    return deformations;
}

template<class Model>
std::vector<typename SkinMixerData<Model>::DualQuaternion> SkinMixerData<Model>::computeDeformation(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->computeDeformation(entry);
}

template<class Model>
std::vector<typename SkinMixerData<Model>::DualQuaternion> SkinMixerData<Model>::computeDeformation(Entry& entry)
{
    std::unordered_set<Index> involvedEntries;

    std::vector<Index> relatedActions = this->relatedActions(entry);
    for (Index aId : relatedActions) {
        const Action& action = this->action(aId);

        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            if (action.entry2 == entry.id) {
                involvedEntries.insert(action.entry1);
                involvedEntries.insert(action.entry2);
            }
        }
    }

    if (involvedEntries.empty()) {
        return computeJointDeformation(entry);
    }
    else {
        std::vector<std::vector<DualQuaternion>> entriesDeformations = computeDeformations();
        return entriesDeformations[entry.id];
    }
}

template<class Model>
std::vector<std::vector<typename SkinMixerData<Model>::DualQuaternion>> SkinMixerData<Model>::computeDeformations()
{
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::Transformation SkeletonTransformation;
    typedef typename Skeleton::Scalar SkeletonScalar;

    //Resulting deformations
    std::vector<std::vector<DualQuaternion>> newDeformations;

    std::vector<std::vector<Translation>> translations(this->entryNumber(), std::vector<Translation>());
    std::vector<std::unordered_set<JointId>> deformedJoints(this->entryNumber());
    std::vector<Skeleton> deformedSkeletons(this->entryNumber());

    std::vector<std::vector<DualQuaternion>> originalDeformations(this->entryNumber(), std::vector<DualQuaternion>());
    std::vector<Skeleton> originalSkeletons(this->entryNumber());

    //Initialize deformations of the involved entries
    for (Index eId = 0; eId < this->entryNumber(); ++eId) {
        const Entry& entry = this->entry(eId);
        const Model* model = entry.model;

        originalDeformations[eId] = computeJointDeformation(eId);
        originalSkeletons[eId] = model->skeleton;

        //Apply transformations to skeleton
        if (!originalDeformations[eId].empty()) {
            #pragma omp parallel for
            for (JointId jId = 0; jId < originalSkeletons[eId].jointNumber(); ++jId) {
                const DualQuaternion& dq = originalDeformations[eId][jId];

                const Affine& t = dq.affineTransformation();
                if (!t.isApprox(Affine::Identity())) {
                    SkeletonTransformation r = t * originalSkeletons[eId].jointBindPose(jId);
                    originalSkeletons[eId].setJointBindPose(jId, r);
                }
            }
        }

        deformedSkeletons[eId] = originalSkeletons[eId];
    }

    //Copy original deformations
    newDeformations = originalDeformations;

    //Get deformation for each action
    for (Index aId = 0; aId < this->actionNumber(); ++aId) {
        const Action& action = this->action(aId);

        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            const Index& eId1 = action.entry1;
            const Index& eId2 = action.entry2;

            const Index& jId1 = action.joint1;
            const Index& jId2 = action.joint2;

            const Entry& entry2 = this->entry(eId2);
            const Model* model2 = entry2.model;

            //Initialize translations
            if (translations[eId2].empty()) {
                translations[eId2].resize(model2->skeleton.jointNumber(), Translation::Identity());
            }

            //Get translations
            nvl::Point3<SkeletonScalar> v1 = deformedSkeletons[eId1].jointBindPose(jId1) * deformedSkeletons[eId1].originPoint();
            nvl::Point3<SkeletonScalar> v2 = originalSkeletons[eId2].jointBindPose(jId2) * originalSkeletons[eId2].originPoint();
            nvl::Vector3<SkeletonScalar> translateVector = v1 - v2;
            nvl::Translation3d translateTransform(translateVector);

            //Set translations and fill deformed joints
            std::vector<JointId> descendantJoints = nvl::skeletonJointDescendants(model2->skeleton, jId2);
            descendantJoints.push_back(jId2);
            for (JointId dId : descendantJoints) {
                translations[eId2][dId] = translateTransform;
                deformedJoints[eId2].insert(dId);
            }

            if (!deformedJoints[eId2].empty()) {
                //Propagation iterations
                const unsigned int propagationIterations = static_cast<unsigned int>(model2->skeleton.jointNumber() * 10);
                const unsigned int deformedNeighbors = 1;

                //Smoothing iterations
                const unsigned int smoothingIterations = static_cast<unsigned int>(model2->skeleton.jointNumber() * 20);

                //Smoothed translations
                std::vector<Translation> smoothedTranslations = translations[eId2];

                //Deform neighbors
                std::unordered_set<JointId> translatedJoints = deformedJoints[eId2];
                for (unsigned int it = 0; it < deformedNeighbors; ++it) {
                    for (JointId jId = 0; jId < model2->skeleton.jointNumber(); ++jId) {
                        if (deformedJoints[eId2].find(jId) == deformedJoints[eId2].end()) {
                            std::vector<Translation> values;
                            std::vector<double> weights;

                            if (!model2->skeleton.isRoot(jId)) {
                                const JointId& parentId = model2->skeleton.parentId(jId);
                                if (deformedJoints[eId2].find(parentId) != deformedJoints[eId2].end()) {
                                    values.push_back(smoothedTranslations[parentId]);
                                    weights.push_back(1.0);
                                }
                            }

                            for (const JointId& childId : model2->skeleton.children(jId)) {
                                if (deformedJoints[eId2].find(childId) != deformedJoints[eId2].end()) {
                                    values.push_back(smoothedTranslations[childId]);
                                    weights.push_back(1.0);
                                }
                            }

                            if (!values.empty()) {
                                assert(values.size() == weights.size());
                                Translation interpolated;
                                if (values.size() > 1) {
                                    nvl::normalize(weights);
                                    interpolated = nvl::interpolateTranslationLinear(values, weights);
                                }
                                else {
                                    interpolated = values[0];
                                }

                                smoothedTranslations[jId] = interpolated;
                                translatedJoints.insert(jId);
                            }
                        }
                    }
                }

                //Propagate
                std::unordered_set<JointId> propagatedJoints = translatedJoints;
                for (unsigned int it = 0; it < propagationIterations; ++it) {
                    bool found = false;
                    for (JointId jId = 0; jId < model2->skeleton.jointNumber() && !found; ++jId) {
                        if (propagatedJoints.find(jId) == propagatedJoints.end()) {
                            std::vector<Translation> values;
                            std::vector<double> weights;

                            if (!model2->skeleton.isRoot(jId)) {
                                const JointId& parentId = model2->skeleton.parentId(jId);
                                if (propagatedJoints.find(parentId) != propagatedJoints.end()) {
                                    values.push_back(smoothedTranslations[parentId]);
                                    weights.push_back(1.0);
                                }
                            }

                            for (const JointId& childId : model2->skeleton.children(jId)) {
                                if (propagatedJoints.find(childId) != propagatedJoints.end()) {
                                    values.push_back(smoothedTranslations[childId]);
                                    weights.push_back(1.0);
                                }
                            }

                            if (!values.empty()) {
                                assert(values.size() == weights.size());
                                Translation interpolated;
                                if (values.size() > 1) {
                                    nvl::normalize(weights);
                                    interpolated = nvl::interpolateTranslationLinear(values, weights);
                                }
                                else {
                                    interpolated = values[0];
                                }

                                smoothedTranslations[jId] = interpolated;
                                propagatedJoints.insert(jId);

                                found = true;
                            }
                        }
                    }
                }

                //Laplacian on other joints
                for (unsigned int it = 0; it < smoothingIterations; ++it) {
                    std::vector<Translation> copyTranslation = smoothedTranslations;

                    for (JointId jId = 0; jId < model2->skeleton.jointNumber(); ++jId) {
                        if (translatedJoints.find(jId) == translatedJoints.end()) {
                            std::vector<Translation> values;
                            std::vector<double> weights;

                            values.push_back(copyTranslation[jId]);
                            weights.push_back(1.0);

                            if (!model2->skeleton.isRoot(jId)) {
                                const JointId& parentId = model2->skeleton.parentId(jId);
                                values.push_back(copyTranslation[parentId]);
                                weights.push_back(1.0);
                            }

                            for (const JointId& childId : model2->skeleton.children(jId)) {
                                values.push_back(copyTranslation[childId]);
                                weights.push_back(1.0);
                            }

                            if (!values.empty()) {
                                assert(values.size() == weights.size());
                                Translation interpolated;
                                if (values.size() > 1) {
                                    nvl::normalize(weights);
                                    interpolated = nvl::interpolateTranslationLinear(values, weights);
                                }
                                else {
                                    interpolated = values[0];
                                }

                                smoothedTranslations[jId] = interpolated;
                            }
                        }
                    }
                }

                if (newDeformations[eId2].empty()) {
                    newDeformations[eId2].resize(model2->skeleton.jointNumber(), DualQuaternion::Identity());
                }

                for (Index jId = 0; jId < model2->skeleton.jointNumber(); ++jId) {
                    if (!originalDeformations.empty()) {
                        newDeformations[eId2][jId] = DualQuaternion(Rotation::Identity(), smoothedTranslations[jId]) * originalDeformations[eId2][jId];
                    }
                    else {
                        newDeformations[eId2][jId] = DualQuaternion(Rotation::Identity(), smoothedTranslations[jId]);
                    }
                }

                //Apply transformations to skeleton
                deformedSkeletons[eId2] = model2->skeleton;
                #pragma omp parallel for
                for (JointId jId = 0; jId < deformedSkeletons[eId2].jointNumber(); ++jId) {
                    const DualQuaternion& dq = newDeformations[eId2][jId];

                    const Affine& t = dq.affineTransformation();
                    if (!t.isApprox(Affine::Identity())) {
                        SkeletonTransformation r = t * deformedSkeletons[eId2].jointBindPose(jId);
                        deformedSkeletons[eId2].setJointBindPose(jId, r);
                    }
                }
            }
        }
    }

    return newDeformations;
}

template<class Model>
void SkinMixerData<Model>::deformModel(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->deformModel(entry);
}

template<class Model>
void SkinMixerData<Model>::deformModel(Entry& entry)
{
    Model* model = entry.model;

    std::vector<DualQuaternion> deformations = computeDeformation(entry);

    if (!deformations.empty()) {
        nvl::modelDeformDualQuaternionSkinning(*model, deformations, false, true);
    }
}

template<class Model>
void SkinMixerData<Model>::deformModels()
{
    std::vector<std::vector<DualQuaternion>> deformations = computeDeformations();

    for (Index eId = 0; eId < entryNumber(); ++eId) {
        if (!deformations[eId].empty()) {
            Entry& entry = this->entry(eId);
            Model* model = entry.model;

            nvl::modelDeformDualQuaternionSkinning(*model, deformations[eId], false, true);
        }
    }
}

template<class Model>
void SkinMixerData<Model>::removeNonStandardTransformationsFromModel(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->removeNonStandardTransformationsFromModel(entry);
}

template<class Model>
void SkinMixerData<Model>::removeNonStandardTransformationsFromModel(Entry& entry)
{
     nvl::modelRemoveNonStandardTransformations(*entry.model);
}

template<class Model>
void SkinMixerData<Model>::removeNonStandardTransformationsFromModels()
{
    //Remove non standard deformations from models
    for (Entry entry : this->entries()) {
        removeNonStandardTransformationsFromModel(entry);
    }
}

template<class Model>
void SkinMixerData<Model>::Entry::clear()
{
    id = nvl::NULL_ID;

    model = nullptr;

    birth.clear();

    blendingAnimationIds.clear();
    blendingAnimationModes.clear();
    blendingAnimationWeights.clear();
    blendingAnimationSpeeds.clear();
}

template<class Model>
void SkinMixerData<Model>::SelectInfo::clear()
{
    vertex.clear();
    joint.clear();
}

template<class Model>
void SkinMixerData<Model>::BirthInfo::clear()
{
    vertex.clear();
    joint.clear();
    mergeJoints.clear();
}

template<class Model>
void SkinMixerData<Model>::Action::clear()
{
    operation = OperationType::NONE;
    entry1 = nvl::NULL_ID;
    entry2 = nvl::NULL_ID;
    joint1 = nvl::NULL_ID;
    joint2 = nvl::NULL_ID;

    select1.clear();
    select2.clear();

    void clear();
}

template<class Model>
std::vector<nvl::Index> SkinMixerData<Model>::relatedActions(const Index& entryId)
{
    const Entry& currentEntry = this->entry(entryId);
    return relatedActions(currentEntry);
}

template<class Model>
std::vector<nvl::Index> SkinMixerData<Model>::relatedActions(const Entry& entry)
{
    std::vector<Index> actions;
    for (Index aId = 0; aId < this->actionNumber(); ++aId) {
        Action& action = this->action(aId);

        if (action.entry1 == entry.id || action.entry2 == entry.id) {
            actions.push_back(aId);
        }
    }

    return actions;
}

template<class Model>
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Index& entryId)
{
    const Entry& currentEntry = this->entry(entryId);
    return computeGlobalSelectInfo(currentEntry);
}

template<class Model>
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Entry& entry)
{
    SelectInfo globalSelectInfo;

    std::vector<Index> actions = this->relatedActions(entry);

    if (actions.empty()) {
        globalSelectInfo.vertex.resize(entry.model->mesh.nextVertexId(), 1.0);
        globalSelectInfo.joint.resize(entry.model->skeleton.jointNumber(), 1.0);

        return globalSelectInfo;
    }

    Action& action = this->action(actions[0]);
    if (action.entry1 == entry.id) {
        globalSelectInfo = action.select1;
    }
    else {
        assert(action.entry2 == entry.id);
        globalSelectInfo = action.select2;
    }

    for (Index rId = 1; rId < actions.size(); ++rId) {
        Action& action = this->action(actions[rId]);

        if (action.operation == OperationType::REMOVE) {
            assert(action.entry1 != nvl::NULL_ID);

            if (action.entry1 == entry.id) {
                const SelectInfo& selectInfo1 = action.select1;
                for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                    if (selectInfo1.vertex[vId] < 1.0) {
                        globalSelectInfo.vertex[vId] = std::min(globalSelectInfo.vertex[vId], selectInfo1.vertex[vId]);
                    }
                }
                for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                    if (selectInfo1.joint[jId] < 1.0) {
                        globalSelectInfo.joint[jId] = std::min(globalSelectInfo.joint[jId], selectInfo1.joint[jId]);
                    }
                }
            }
        }
        else if (action.operation == OperationType::DETACH) {
            assert(action.entry1 != nvl::NULL_ID);

            if (action.entry1 == entry.id) {
                const SelectInfo& selectInfo1 = action.select1;
                for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                    if (selectInfo1.vertex[vId] > 0.0) {
                        globalSelectInfo.vertex[vId] = std::max(globalSelectInfo.vertex[vId], selectInfo1.vertex[vId]);
                    }
                }
                for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                    if (selectInfo1.joint[jId] > 0.0) {
                        globalSelectInfo.joint[jId] = std::max(globalSelectInfo.joint[jId], selectInfo1.joint[jId]);
                    }
                }
            }
        }
        else if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            assert(action.entry1 != nvl::NULL_ID);
            assert(action.entry2 != nvl::NULL_ID);

            if (action.entry1 == entry.id) {
                const SelectInfo& selectInfo1 = action.select1;
                for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                    if (selectInfo1.vertex[vId] < 1.0) {
                        globalSelectInfo.vertex[vId] = std::min(globalSelectInfo.vertex[vId], selectInfo1.vertex[vId]);
                    }
                }
                for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                    if (selectInfo1.joint[jId] < 1.0) {
                        globalSelectInfo.joint[jId] = std::min(globalSelectInfo.joint[jId], selectInfo1.joint[jId]);
                    }
                }
            }

            if (action.entry2 == entry.id) {
                const SelectInfo& selectInfo2 = action.select2;
                for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                    if (selectInfo2.vertex[vId] > 0.0) {
                        globalSelectInfo.vertex[vId] = std::max(globalSelectInfo.vertex[vId], selectInfo2.vertex[vId]);
                    }
                }
                for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                    if (selectInfo2.joint[jId] > 0.0) {
                        globalSelectInfo.joint[jId] = std::max(globalSelectInfo.joint[jId], selectInfo2.joint[jId]);
                    }
                }
            }
        }
    }

    return globalSelectInfo;
}

template<class Model>
void SkinMixerData<Model>::clear()
{
    vEntries.clear();
    vModelMap.clear();
    vActions.clear();
}

template<class Model>
void SkinMixerData<Model>::clearActions()
{
    this->vActions.clear();
}

}
