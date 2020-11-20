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
        if (entry.relatedActions.size() > 0) {
            nvl::modelApplyTransformation(*entry.model, entry.frame);
            cluster.push_back(entry.id);
        }
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
void replace(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry1 = data.entryFromModel(model1);
    Entry& entry2 = data.entryFromModel(model2);

    model1->mesh.computeNormals();
    model2->mesh.computeNormals();

    std::vector<double> vertexValues1;
    std::vector<double> jointValues1;
    std::vector<double> vertexValues2;
    std::vector<double> jointValues2;
    skinmixer::computeReplaceSelectValues(*model1, *model2, targetJoint1, targetJoint2, functionSmoothingIterations, rigidity, hardness1, hardness2, vertexValues1, jointValues1, vertexValues2, jointValues2);

    Action action;
    action.operation = OperationType::REPLACE;
    action.entry1 = entry1.id;
    action.entry2 = entry2.id;
    action.joint1 = targetJoint1;
    action.joint2 = targetJoint2;
    action.hardness1 = hardness1;
    action.hardness2 = hardness2;
    action.select1.vertex = vertexValues1;
    action.select1.joint = jointValues1;
    action.select2.vertex = vertexValues2;
    action.select2.joint = jointValues2;

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
        const double rigidity,
        const double hardness)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeRemoveSelectValues(*model, targetJoint, functionSmoothingIterations, rigidity, hardness, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::REMOVE;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_INDEX;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);
}

template<class Model>
void detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeDetachSelectValues(*model, targetJoint, smoothingIterations, rigidity, hardness, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::DETACH;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_INDEX;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);
}

}
