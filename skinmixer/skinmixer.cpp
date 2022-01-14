#include "skinmixer.h"

#include "skinmixer/skinmixer_utilities.h"
#include "skinmixer/skinmixer_select.h"
#include "skinmixer/skinmixer_blend_skeletons.h"
#include "skinmixer/skinmixer_blend_surfaces.h"
#include "skinmixer/skinmixer_blend_skinningweights.h"
#include "skinmixer/skinmixer_blend_animations.h"

#include <nvl/models/algorithms/model_transformations.h>
#include <nvl/models/algorithms/mesh_normals.h>

#include <nvl/structures/containers/disjoint_set.h>

#include <chrono>

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> mix(
        SkinMixerData<Model>& data,
        const MixParameters& par)
{
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;

#ifdef GLOBAL_TIMES
    chrono::steady_clock::time_point start;
    long duration;
    long totalDuration = 0;
#endif

    //Remove non standard transformations and deform the models
    data.removeNonStandardTransformationsFromModels();
    data.deformModels();

    //Resulting entries
    std::vector<Index> newEntries;


    //Find clusters
    nvl::DisjointSet<Index> ds;
    for (const Action& action : data.actions()) {
         ds.insert(action.entry1);
        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            ds.insert(action.entry2);
            ds.merge(action.entry1, action.entry2);
        }
    }
    std::vector<std::vector<Index>> clusters = ds.computeSets();

    for (const std::vector<Index>& cluster : clusters) {
        std::cout << std::endl << "--------- Cluster: ";
        for (Index cId : cluster) {
            std::cout << cId << " ";
        }
        std::cout << "---------" << std::endl;

        //New model
        Model* resultModel = new Model();

        Index newEntryId = data.addEntry(resultModel);
        Entry& resultEntry = data.entry(newEntryId);

        newEntries.push_back(newEntryId);

#ifdef GLOBAL_TIMES
        start = chrono::steady_clock::now();
        std::cout << std::endl << "*** SURFACE BLENDING ***" << std::endl;
#endif

        //Blend surface
        blendSurfaces(
                    data,
                    cluster,
                    resultEntry,
                    par.mixMode,
                    par.blendColorsFromTextures,
                    par.smoothingBorderIterations,
                    par.smoothingBorderThreshold,
                    par.smoothingInnerIterations,
                    par.smoothingInnerAlpha,
                    par.smoothingResultIterations,
                    par.voxelSize,
                    par.voxelDistance);

#ifdef GLOBAL_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << ">>>>> SURFACE BLENDING: " << duration << " ms" << std::endl;
        totalDuration += duration;
#endif



#ifdef GLOBAL_TIMES
        start = chrono::steady_clock::now();
        std::cout << std::endl << "*** SKELETON BLENDING ***" << std::endl;
#endif

        //Blend skeleton
        blendSkeletons(data, cluster, resultEntry);

#ifdef GLOBAL_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << ">>>>> SKELETON BLENDING: " << duration << " ms" << std::endl;
        totalDuration += duration;
#endif



#ifdef GLOBAL_TIMES
        start = chrono::steady_clock::now();
        std::cout << std::endl << "*** SKINNING WEIGHT BLENDING ***" << std::endl;
#endif

        //Blend skinning weights
        blendSkinningWeights(data, cluster, resultEntry);

#ifdef GLOBAL_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << ">>>>> SKINNING WEIGHT: " << duration << " ms" << std::endl;
        totalDuration += duration;
#endif



#ifdef GLOBAL_TIMES
        start = chrono::steady_clock::now();
        std::cout << std::endl << "*** INITIALIZE ANIMATION WEIGHTS ***" << std::endl;
#endif

        //Initialize animation weights
        initializeAnimationWeights(data, cluster, resultEntry);

#ifdef GLOBAL_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << ">>>>> INITIALIZE ANIMATION WEIGHTS: " << duration << " ms" << std::endl;
        totalDuration += duration;
#endif


#ifdef GLOBAL_TIMES
        std::cout << std::endl << "TOTAL DURATION: " << totalDuration << " ms" << std::endl;
#endif

        std::cout << std::endl;
    }

    data.clearActions();

    return newEntries;
}

template<class Model>
void mixAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        std::vector<std::pair<nvl::Index, nvl::Index>>& resultAnimations,
        const MixAnimationParameters& parameters)
{
#ifdef GLOBAL_TIMES
    chrono::steady_clock::time_point start;
    long duration;

    start = chrono::steady_clock::now();
    std::cout << std::endl << "*** ANIMATION BLENDING ***" << std::endl;
#endif

    //Blending animations
    blendAnimations(
                data,
                entry,
                resultAnimations,
                parameters.samplingFPS,
                parameters.rotationWeight,
                parameters.globalWeight,
                parameters.localWeight,
                parameters.globalDerivativeWeight,
                parameters.localDerivativeWeight,
                parameters.windowSize,
                parameters.windowMainWeight,
                parameters.smoothingIterations,
                parameters.smoothingThreshold);

#ifdef GLOBAL_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << ">>>>> ANIMATION BLENDING: " << duration << " ms" << std::endl;
#endif
}

template<class Model>
nvl::Index replace(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const ReplaceMode& replaceMode,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename nvl::Index Index;

    Entry& entry1 = data.entryFromModel(model1);
    Entry& entry2 = data.entryFromModel(model2);

    model1->mesh.computeNormals();
    model2->mesh.computeNormals();

    std::vector<double> vertexValues1;
    std::vector<double> jointValues1;
    std::vector<double> vertexValues2;
    std::vector<double> jointValues2;
    skinmixer::computeReplaceSelectValues(*model1, *model2, targetJoint1, targetJoint2, weightSmoothingIterations, rigidity, hardness1, hardness2, includeParent1, includeParent2, vertexValues1, jointValues1, vertexValues2, jointValues2);

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
    action.replaceMode = replaceMode;

    Index actionId = data.addAction(action);

    return actionId;
}

template<class Model>
nvl::Index attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename nvl::Index Index;

    Entry& entry1 = data.entryFromModel(model1);
    Entry& entry2 = data.entryFromModel(model2);

    model1->mesh.computeNormals();
    model2->mesh.computeNormals();

    std::vector<double> vertexValues1;
    std::vector<double> jointValues1;
    std::vector<double> vertexValues2;
    std::vector<double> jointValues2;
    skinmixer::computeAttachSelectValues(*model1, *model2, (includeParent1 ? model1->skeleton.parentId(targetJoint1) : targetJoint1), targetJoint2, weightSmoothingIterations, rigidity, hardness2, includeParent2, vertexValues1, jointValues1, vertexValues2, jointValues2);

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

    Index actionId = data.addAction(action);
    return actionId;
}

template<class Model>
nvl::Index remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename nvl::Index Index;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeRemoveSelectValues(*model, targetJoint, weightSmoothingIterations, rigidity, hardness, includeParent, 0.5, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::REMOVE;
    action.entry1 = entry.id;
    action.entry2 = nvl::NULL_ID;
    action.joint1 = includeParent ? model->skeleton.parentId(targetJoint) : targetJoint;
    action.joint2 = nvl::NULL_ID;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    Index actionId = data.addAction(action);

    return actionId;
}

template<class Model>
nvl::Index detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename nvl::Index Index;

    Entry& entry = data.entryFromModel(model);

    std::vector<double> vertexValues;
    std::vector<double> jointValues;
    computeDetachSelectValues(*model, targetJoint, weightSmoothingIterations, rigidity, hardness, includeParent, 0.5, vertexValues, jointValues);

    Action action;
    action.operation = OperationType::DETACH;
    action.entry1 = entry.id;
    action.entry2 = nvl::NULL_ID;
    action.joint1 = includeParent ? model->skeleton.parentId(targetJoint) : targetJoint;
    action.joint2 = nvl::NULL_ID;
    action.hardness1 = hardness;
    action.select1.vertex = vertexValues;
    action.select1.joint = jointValues;

    Index actionId = data.addAction(action);

    return actionId;
}

}
