#include "skinmixer.h"

#include "skinmixer/skinmixer_operation.h"
#include "skinmixer/skinmixer_utilities.h"
#include "skinmixer/skinmixer_fuzzy.h"
#include "skinmixer/skinmixer_blend.h"

#include <nvl/models/model_transformations.h>
#include <nvl/models/mesh_normals.h>

namespace skinmixer {

template<class Model>
std::vector<Model*> mix(
        SkinMixerData<Model>& data)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename Model::Mesh Mesh;

    std::vector<Model*> newModels;

    std::vector<Model*> models;
    std::vector<std::vector<float>> vertexFuzzyValue;
    std::vector<std::vector<float>> jointFuzzyValue;

    //TODO CLUSTERS
    for (Entry entry : data.entries()) {
        models.push_back(entry.model);
        vertexFuzzyValue.push_back(entry.vertexFuzzyValue);
        jointFuzzyValue.push_back(entry.jointFuzzyValue);
    }

    Model* resultModel = new Model();

    std::vector<std::pair<nvl::Index, typename Mesh::VertexId>> birthVertex;
    std::vector<std::pair<nvl::Index, typename Mesh::FaceId>> birthFace;
    blendModels(models, vertexFuzzyValue, resultModel, birthVertex, birthFace);

    newModels.push_back(resultModel);

    return newModels;
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
    typedef typename SkinMixerData<Model>::Action Action;

    nvl::modelApplyTransformation(*model1, transformation1);
    nvl::meshUpdateFaceNormals(model1->mesh);
    nvl::meshUpdateVertexNormals(model1->mesh);

    nvl::modelApplyTransformation(*model2, transformation2);
    nvl::meshUpdateFaceNormals(model2->mesh);
    nvl::meshUpdateVertexNormals(model2->mesh);

    Action action;
    action.operation = OperationType::ATTACH;
    action.model1 = model1;
    action.model2 = model2;
    action.joint1 = targetJoint1;
    action.joint2 = targetJoint2;
    data.addAction(action);
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

    Entry& entry = data.entry(model);

    removeFuzzy(*model, targetJoint, entry.vertexFuzzyValue, entry.jointFuzzyValue, functionSmoothingIterations, offset, rigidity);

    Action action;
    action.operation = OperationType::REMOVE;
    action.model1 = model;
    action.model2 = nullptr;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_ID;
    data.addAction(action);
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

    Entry& entry = data.entry(model);

    detachFuzzy(*model, targetJoint, entry.vertexFuzzyValue, entry.jointFuzzyValue, functionSmoothingIterations, offset, rigidity);

    Action action;
    action.operation = OperationType::DETACH;
    action.model1 = model;
    action.model2 = nullptr;
    action.joint1 = targetJoint;
    action.joint2 = nvl::MAX_ID;
    data.addAction(action);
}

}
