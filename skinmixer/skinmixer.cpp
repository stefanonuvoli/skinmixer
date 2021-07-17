#include "skinmixer.h"

#include "skinmixer/skinmixer_operation.h"
#include "skinmixer/skinmixer_utilities.h"
#include "skinmixer/skinmixer_select.h"
#include "skinmixer/skinmixer_blend_skeletons.h"
#include "skinmixer/skinmixer_blend_surfaces.h"
#include "skinmixer/skinmixer_blend_skinningweights.h"
#include "skinmixer/skinmixer_blend_animations.h"

#include <nvl/models/model_transformations.h>
#include <nvl/models/model_deformation.h>
#include <nvl/models/mesh_normals.h>

#include <chrono>

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> mix(
        SkinMixerData<Model>& data)
{
    typedef nvl::Index Index;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename Model::Mesh Mesh;

    chrono::steady_clock::time_point start;

    for (Entry entry : data.entries()) {
        if (entry.relatedActions.size() > 0) {
            data.computeDeformation(entry);
            if (!entry.deformation.empty()) {
                nvl::modelDeformDualQuaternionSkinning(*entry.model, entry.deformation);
            }
        }
    }

    std::vector<Index> newEntries;

    //Blend surface
    start = chrono::steady_clock::now();
    blendSurfaces(data, newEntries);
    std::cout << "Surface blended in " << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() << " ms" << std::endl;


    //Blend skeleton
    start = chrono::steady_clock::now();
    blendSkeletons(data, newEntries);
    std::cout << "Skeleton blended in " << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() << " ms" << std::endl;


    //Blend skinning weights
    start = chrono::steady_clock::now();
    blendSkinningWeights(data, newEntries);
    std::cout << "Skinning weight blended in " << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() << " ms" << std::endl;

    //Blend skinning weights
    start = chrono::steady_clock::now();
    initializeAnimationWeights(data, newEntries);
    std::cout << "Animation weight initialized in " << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() << " ms" << std::endl;


    data.clearActions();

    return newEntries;
}

template<class Model>
void mixAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        nvl::Index& targetAnimationId)
{
    chrono::steady_clock::time_point start;

    //Blending animations
    start = chrono::steady_clock::now();
    return blendAnimations(data, entry, targetAnimationId);
    std::cout << "Blended animations in " << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() << " ms" << std::endl;

}

template<class Model>
nvl::Index replace(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2,
        const nvl::Affine3d& vActionRotation,
        const nvl::Translation3d& vActionTranslation)
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
    skinmixer::computeReplaceSelectValues(*model1, *model2, targetJoint1, targetJoint2, functionSmoothingIterations, rigidity, hardness1, hardness2, includeParent1, includeParent2, vertexValues1, jointValues1, vertexValues2, jointValues2);

    Action action;
    action.operation = OperationType::REPLACE;
    action.entry1 = entry1.id;
    action.entry2 = entry2.id;
    action.joint1 = includeParent1 ? model1->skeleton.parentId(targetJoint1) : targetJoint1;
    action.joint2 = includeParent2 ? model2->skeleton.parentId(targetJoint2) : targetJoint2;
    action.hardness1 = hardness1;
    action.hardness2 = hardness2;
    action.select1.vertex = vertexValues1;
    action.select1.joint = jointValues1;
    action.select2.vertex = vertexValues2;
    action.select2.joint = jointValues2;
    action.rotation2 = vActionRotation;
    action.translation2 = vActionTranslation;

    nvl::Index actionId = data.addAction(action);
    entry1.relatedActions.push_back(actionId);
    entry2.relatedActions.push_back(actionId);

    return actionId;
}

template<class Model>
nvl::Index attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2,
        const nvl::Affine3d& vActionRotation,
        const nvl::Translation3d& vActionTranslation)
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
    skinmixer::computeAttachSelectValues(*model1, *model2, (includeParent1 ? model1->skeleton.parentId(targetJoint1) : targetJoint1), targetJoint2, functionSmoothingIterations, rigidity, hardness2, includeParent2, vertexValues1, jointValues1, vertexValues2, jointValues2);

    Action action;
    action.operation = OperationType::ATTACH;
    action.entry1 = entry1.id;
    action.entry2 = entry2.id;
    action.joint1 = includeParent1 ? model1->skeleton.parentId(targetJoint1) : targetJoint1;
    action.joint2 = includeParent2 ? model2->skeleton.parentId(targetJoint2) : targetJoint2;
    action.hardness2 = hardness2;
    action.select1.vertex = vertexValues1;
    action.select1.joint = jointValues1;
    action.select2.vertex = vertexValues2;
    action.select2.joint = jointValues2;
    action.rotation2 = vActionRotation;
    action.translation2 = vActionTranslation;

    nvl::Index actionId = data.addAction(action);
    entry1.relatedActions.push_back(actionId);
    entry2.relatedActions.push_back(actionId);

    return actionId;
}

template<class Model>
nvl::Index remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeRemoveSelectValues(*model, targetJoint, functionSmoothingIterations, rigidity, hardness, includeParent, 0.5, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::REMOVE;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = includeParent ? model->skeleton.parentId(targetJoint) : targetJoint;
    action.joint2 = nvl::MAX_INDEX;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);

    return actionId;
}

template<class Model>
nvl::Index detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeDetachSelectValues(*model, targetJoint, smoothingIterations, rigidity, hardness, includeParent, 0.5, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::DETACH;
    action.entry1 = entry.id;
    action.entry2 = nvl::MAX_INDEX;
    action.joint1 = includeParent ? model->skeleton.parentId(targetJoint) : targetJoint;
    action.joint2 = nvl::MAX_INDEX;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    nvl::Index actionId = data.addAction(action);
    entry.relatedActions.push_back(actionId);

    return actionId;
}

}
