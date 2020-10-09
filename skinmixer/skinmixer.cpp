#include "skinmixer.h"

#include "skinmixer/skinmixer_operation.h"
#include "skinmixer/skinmixer_utilities.h"
#include "skinmixer/skinmixer_select.h"
#include "skinmixer/skinmixer_blend_skeletons.h"
#include "skinmixer/skinmixer_blend_surfaces.h"
#include "skinmixer/skinmixer_blend_skinningweights.h"
#include "skinmixer/skinmixer_blend_animations.h"

#include <nvl/models/model_transformations.h>
#include <nvl/models/mesh_normals.h>

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> mix(
        SkinMixerData<Model>& data)
{
    typedef nvl::Index Index;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename Model::Mesh Mesh;

    std::vector<Index> newEntries;

    std::vector<std::vector<Index>> entryClusters;

    //TODO: clusters by actions, for now we insert every model
    std::vector<Index> cluster;
    for (Entry entry : data.entries()) {
        cluster.push_back(entry.id);
    }
    entryClusters.push_back(cluster);

    for (const std::vector<Index>& cluster : entryClusters) {
        Model* resultModel = new Model();
        Index newEntryId = data.addEntry(resultModel);

        Entry& entry = data.entry(newEntryId);
        entry.birth.entries = cluster;

        blendSurfaces(data, cluster, entry);
        blendSkeletons(data, cluster, entry);

        blendSkinningWeights(data, entry);

        initializeAnimationWeights(data, entry);

        newEntries.push_back(newEntryId);
    }

    return newEntries;
}

template<class Model>
void mixAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        nvl::Index& targetAnimationId)
{
    return blendAnimations(data, entry, targetAnimationId);
}

template<class Model>
void attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& transformation1,
        const nvl::Affine3d& transformation2)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry1 = data.entryFromModel(model1);
    Entry& entry2 = data.entryFromModel(model2);

    nvl::modelApplyTransformation(*model1, transformation1);
    nvl::meshUpdateFaceNormals(model1->mesh);
    nvl::meshUpdateVertexNormals(model1->mesh);

    nvl::modelApplyTransformation(*model2, transformation2);
    nvl::meshUpdateFaceNormals(model2->mesh);
    nvl::meshUpdateVertexNormals(model2->mesh);

    Action action;
    action.operation = OperationType::ATTACH;
    action.entry1 = entry1.id;
    action.entry2 = entry2.id;
    action.joint1 = targetJoint1;
    action.joint2 = targetJoint2;

    nvl::Index actionId = data.addAction(action);
    entry1.relatedActions.push_back(actionId);
    entry2.relatedActions.push_back(actionId);
}

template<class Model>
void remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    computeRemoveSelectValues(*model, targetJoint, functionSmoothingIterations, offset, rigidity, entry.select.vertex, entry.select.joint);

    Action action;
    action.operation = OperationType::REMOVE;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_INDEX;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);
}

template<class Model>
void detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    computeDetachSelectValues(*model, targetJoint, functionSmoothingIterations, offset, rigidity, entry.select.vertex, entry.select.joint);

    Action action;
    action.operation = OperationType::DETACH;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_INDEX;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);
}

}
