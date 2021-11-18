#include "skinmixer_data.h"

#include <nvl/models/algorithms/animation_skinning.h>
#include <nvl/models/algorithms/skeleton_adjacencies.h>
#include <nvl/math/interpolation.h>
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
    for (Entry& entry : vEntries) {
        std::vector<Index>::iterator it = std::find(entry.relatedActions.begin(), entry.relatedActions.end(), index);
        if (it != entry.relatedActions.end()) {
            entry.relatedActions.erase(it);
        }
    }
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
void SkinMixerData<Model>::computeDeformation(const Index& entryId)
{
    Entry& entry = this->entry(entryId);
    return this->computeDeformation(entry);
}

template<class Model>
void SkinMixerData<Model>::computeDeformation(Entry& entry)
{
    Model* model = entry.model;
    std::vector<nvl::DualQuaterniond>& deformation = entry.deformation;
    deformation.clear();

    std::vector<nvl::Affine3d> transformations(model->skeleton.jointNumber(), nvl::Affine3d::Identity());
    std::vector<nvl::Translation3d> translations(model->skeleton.jointNumber(), nvl::Translation3d::Identity());

    std::unordered_set<JointId> deformedJoints;

    std::vector<std::unordered_set<Index>> vertexInvolvedActions(model->mesh.nextVertexId());
    std::vector<std::unordered_set<Index>> jointInvolvedActions(model->skeleton.jointNumber());

    for (Index aId : entry.relatedActions) {
        const Action& action = this->action(aId);

        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            if (action.entry2 == entry.id) {
                std::vector<JointId> descendantJoints = nvl::skeletonJointDescendants(model->skeleton, action.joint2);
                descendantJoints.push_back(action.joint2);

                for (JointId dId : descendantJoints) {
                    transformations[dId] = action.translation2 * action.rotation2;
                    translations[dId] = action.translation2;

                    deformedJoints.insert(dId);

                    for (VertexId vId = 0; vId < model->mesh.nextVertexId(); ++vId) {
                        if (model->mesh.isVertexDeleted(vId))
                            continue;

                        if (model->skinningWeights.weight(vId, dId) >= MIN_SKINNING_WEIGHT) {
                            vertexInvolvedActions[vId].insert(aId);
                        }
                    }
                }
            }
        }
    }

    if (!deformedJoints.empty()) {
        const int propagationIterations = 1;
        const int smoothingIterations = model->skeleton.jointNumber() * 20;

        //Propagate
        for (int it = 0; it < propagationIterations; ++it) {
            std::vector<nvl::Translation3d> copyTranslation = translations;
            std::unordered_set<JointId> copyDeformedJoints = deformedJoints;

            for (JointId jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
                if (copyDeformedJoints.find(jId) == copyDeformedJoints.end()) {
                    std::vector<nvl::Translation3d> values;

                    if (!model->skeleton.isRoot(jId)) {
                        const JointId& parentId = model->skeleton.parentId(jId);
                        if (copyDeformedJoints.find(parentId) != copyDeformedJoints.end()) {
                            values.push_back(copyTranslation[parentId]);
                        }
                    }
                    for (const JointId& childId : model->skeleton.children(jId)) {
                        if (copyDeformedJoints.find(childId) != copyDeformedJoints.end()) {
                            values.push_back(copyTranslation[childId]);
                        }
                    }

                    if (!values.empty()) {
                        nvl::Vector3d vec(0,0,0);
                        for (Index i = 0; i < values.size(); ++i) {
                            vec += values[i].vector();
                        }
                        vec /= static_cast<double>(values.size());

                        transformations[jId] = nvl::Translation3d(vec);
                        translations[jId] = nvl::Translation3d(vec);
                        deformedJoints.insert(jId);
                    }
                }
            }
        }


        //Deformation based on skinning weights
        for (JointId jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
            if (deformedJoints.find(jId) == deformedJoints.end()) {
                for (VertexId vId = 0; vId < model->mesh.nextVertexId(); ++vId) {
                    if (!vertexInvolvedActions[vId].empty() && model->skinningWeights.weight(vId, jId) >= MIN_SKINNING_WEIGHT) {
                        jointInvolvedActions[jId].insert(vertexInvolvedActions[vId].begin(), vertexInvolvedActions[vId].end());
                    }
                }
            }
        }
        for (JointId jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
            if (!jointInvolvedActions[jId].empty() && deformedJoints.find(jId) == deformedJoints.end()) {
                std::vector<nvl::Translation3d> values;

                for (Index aId : jointInvolvedActions[jId]) {
                    const Action& action = this->action(aId);

                    values.push_back(action.translation2);
                }

                if (!values.empty()) {
                    nvl::Vector3d vec(0,0,0);
                    for (Index i = 0; i < values.size(); ++i) {
                        vec += values[i].vector();
                    }
                    vec /= static_cast<double>(values.size());

                    transformations[jId] = nvl::Translation3d(vec);
                    translations[jId] = nvl::Translation3d(vec);
                    deformedJoints.insert(jId);
                }
            }
        }



        //Laplacian on other joints
        for (int it = 0; it < smoothingIterations; ++it) {
            std::vector<nvl::Translation3d> copyTranslation = translations;

            for (JointId jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
                if (deformedJoints.find(jId) == deformedJoints.end()) {
                    std::vector<nvl::Translation3d> values;

                    values.push_back(copyTranslation[jId]);
                    if (!model->skeleton.isRoot(jId)) {
                        const JointId& parentId = model->skeleton.parentId(jId);
                        values.push_back(copyTranslation[parentId]);
                    }
                    for (const JointId& childId : model->skeleton.children(jId)) {
                        values.push_back(copyTranslation[childId]);
                    }

                    nvl::Vector3d vec(0,0,0);
                    if (values.size() > 1) {
                        for (Index i = 0; i < values.size(); ++i) {
                            vec += values[i].vector();
                        }
                        vec /= static_cast<double>(values.size());
                    }
                    else {
                        vec = values[0].vector();
                    }

                    transformations[jId] = nvl::Translation3d(vec);
                    translations[jId] = nvl::Translation3d(vec);
                }
            }
        }



//        for (int it = 0; it < smoothingIterations; ++it) {
//            for (JointId jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
//                if (deformedJoints.find(jId) == deformedJoints.end()) {
//                    std::vector<nvl::DualQuaterniond> dualQuaternions;

//                    dualQuaternions.push_back(nvl::DualQuaterniond(nvl::Rotation3d(transformations[jId].rotation()), nvl::Translation3d(transformations[jId].translation())));
//                    const JointId& parentId = model->skeleton.parentId(jId);
//                    if (parentId != nvl::NULL_ID) {
//                        dualQuaternions.push_back(nvl::DualQuaterniond(nvl::Rotation3d(transformations[parentId].rotation()), nvl::Translation3d(transformations[parentId].translation())));
//                    }
//                    for (const JointId& childId : model->skeleton.children(jId)) {
//                        dualQuaternions.push_back(nvl::DualQuaterniond(nvl::Rotation3d(transformations[childId].rotation()), nvl::Translation3d(transformations[childId].translation())));
//                    }

//                    std::vector<double> dualQuaternionsWeights(dualQuaternions.size(), 1.0 / dualQuaternions.size());

//                    nvl::DualQuaterniond averageDualQuaternion;
//                    averageDualQuaternion.setZero();

//                    for (Index i = 0; i < dualQuaternions.size(); ++i) {
//                        double weight = dualQuaternionsWeights[i];

//                        const double dot = dualQuaternions[0].rotation().dot(dualQuaternions[i].rotation());
//                        if (dot < 0.0)
//                            weight *= -1;

//                        averageDualQuaternion = averageDualQuaternion + (weight * dualQuaternions[i]);
//                    }
//                    averageDualQuaternion.normalize();

//                    transformations[jId] = averageDualQuaternion.affineTransformation();
//                }
//            }
//        }




        deformation.resize(model->skeleton.jointNumber());
        for (Index jId = 0; jId < model->skeleton.jointNumber(); ++jId) {
            deformation[jId] = nvl::DualQuaterniond(transformations[jId]);
        }
    }
}

template<class Model>
void SkinMixerData<Model>::Entry::clear()
{
    id = nvl::NULL_ID;

    model = nullptr;

    birth.clear();
    relatedActions.clear();

    blendingAnimationIds.clear();
    blendingAnimationModes.clear();
    blendingAnimationWeights.clear();
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
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Entry& entry)
{
    SelectInfo globalSelectInfo;
    globalSelectInfo.vertex.resize(entry.model->mesh.nextVertexId(), 1.0);
    globalSelectInfo.joint.resize(entry.model->skeleton.jointNumber(), 1.0);


    for (Index aId : entry.relatedActions) {
        Action& action = this->action(aId);

        if (action.entry1 == entry.id) {
            const SelectInfo& selectInfo = action.select1;
            for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                globalSelectInfo.vertex[vId] = std::min(globalSelectInfo.vertex[vId], selectInfo.vertex[vId]);
            }
            for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                globalSelectInfo.joint[jId] = std::min(globalSelectInfo.joint[jId], selectInfo.joint[jId]);
            }
        }
        else if (action.entry2 == entry.id) {
            const SelectInfo& selectInfo = action.select2;
            for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                globalSelectInfo.vertex[vId] = std::min(globalSelectInfo.vertex[vId], selectInfo.vertex[vId]);
            }
            for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                globalSelectInfo.joint[jId] = std::min(globalSelectInfo.joint[jId], selectInfo.joint[jId]);
            }
        }
    }

    for (Index aId : entry.relatedActions) {
        Action& action = this->action(aId);

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
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Index& eId)
{
    const Entry& currentEntry = this->entry(eId);

    return computeGlobalSelectInfo(currentEntry);
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
    for (Entry& entry : this->vEntries) {
        entry.relatedActions.clear();
    }
}
}
