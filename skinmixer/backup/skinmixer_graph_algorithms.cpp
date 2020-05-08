#include "skinmixer_graph_algorithms.h"

#include <nvl/models/model_transformations.h>
#include <nvl/models/mesh_normals.h>

namespace skinmixer {

namespace internal {

template<class Model>
bool findOriginalPositionRecursive(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Index& rootNodeId,
        typename Model::Mesh::Point& position);

}

template<class Model>
bool findVertexBirthNode(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const typename Model::Mesh::VertexId& vertexId,
        nvl::Index& birthNode,
        typename Model::Mesh::VertexId& birthVertex)
{
    typedef typename Model::Mesh::VertexId VertexId;
    typedef typename SkinMixerGraph<Model>::Node Node;

    birthNode = nvl::MAX_ID;
    birthVertex = nvl::MAX_ID;

    nvl::Index currentNodeId = nodeId;
    VertexId currentVertexId = vertexId;
    bool isRoot = graph.node(currentNodeId).parents.empty();
    if (isRoot)
        return true;

    bool isNewVertex = false;
    while (!isNewVertex && !isRoot) {
        const Node& currentNode = graph.node(currentNodeId);
        assert(!currentNode.birthVertex.empty());

        nvl::Index birthVertexParentNodeId = currentNode.birthVertexParentNodeId[currentVertexId];
        VertexId birthVertex = currentNode.birthVertex[currentVertexId];

        isNewVertex =
            birthVertexParentNodeId == nvl::MAX_ID ||
            birthVertex == nvl::MAX_ID;

        if (!isNewVertex) {
            currentNodeId = currentNode.parents[birthVertexParentNodeId];
            currentVertexId = birthVertex;

            isRoot = graph.node(currentNodeId).parents.empty();
        }
    }

    if (!isNewVertex) {
        birthNode = currentNodeId;
        birthVertex = currentVertexId;
    }

    return isNewVertex;
}

template<class Model>
bool findJointBirthNode(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const typename Model::Skeleton::JointId& jointId,
        nvl::Index& birthNode,
        typename Model::Skeleton::JointId& birthJoint)
{
    typedef typename Model::Skeleton::JointId JointId;
    typedef typename SkinMixerGraph<Model>::Node Node;

    birthNode = nvl::MAX_ID;
    birthJoint = nvl::MAX_ID;

    nvl::Index currentNodeId = nodeId;
    JointId currentJointId = jointId;
    bool isRoot = graph.node(currentNodeId).parents.empty();
    if (isRoot)
        return true;

    bool isNewJoint = false;
    while (!isNewJoint && !isRoot) {
        const Node& currentNode = graph.node(currentNodeId);
        assert(!currentNode.birthJoint.empty());

        nvl::Index birthJointParentNodeId = currentNode.birthJointParentNodeId[currentJointId];
        JointId birthJoint = currentNode.birthJoint[currentJointId];

        isNewJoint =
            birthJointParentNodeId == nvl::MAX_ID ||
            birthJoint == nvl::MAX_ID;

        if (!isNewJoint) {
            currentNodeId = currentNode.parents[birthJointParentNodeId];
            currentJointId = birthJoint;

            isRoot = graph.node(currentNodeId).parents.empty();
        }
    }

    if (!isNewJoint) {
        birthNode = currentNodeId;
        birthJoint = currentJointId;
    }

    return isNewJoint;
}

template<class Model>
bool findOriginalPosition(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Index& rootNodeId,
        const typename Model::Mesh::Point& position,
        typename Model::Mesh::Point& originalPosition)
{
    originalPosition = position;

    bool found = internal::findOriginalPositionRecursive(graph, nodeId, rootNodeId, originalPosition);

    return found;
}

template<class Model>
void applyTransformationToNode(
        SkinMixerNode<Model>& node,
        const nvl::Affine3d& transformation)
{
    Model* model = node.model;

    //Apply transformation to node
    node.transformation = transformation * node.transformation;

    //Apply transformation to the model
    nvl::modelApplyTransformation(*model, transformation);
    nvl::meshUpdateFaceNormals(model->mesh);
    nvl::meshUpdateVertexNormals(model->mesh);
}

template<class Model>
void applyTransformationToNode(
        SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Affine3d& transformation)
{
    return applyTransformationToNode(graph.node(nodeId), transformation);
}

namespace internal {

template<class Model>
bool findOriginalPositionRecursive(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Index& rootNodeId,
        typename Model::Mesh::Point& position)
{
    typedef typename SkinMixerGraph<Model>::Node Node;
    typedef typename nvl::Index Index;
    typedef typename Model::Mesh::Point Point;

    if (nodeId == rootNodeId) {
        return true;
    }
    else {
        const Node& currentNode = graph.node(nodeId);

        position = currentNode.transformation.inverse() * position;

        const std::vector<Index>& parents = currentNode.parents;

        for (Index parent : parents) {
            Point newPosition = position;
            bool found = findOriginalPositionRecursive(graph, parent, rootNodeId, newPosition);
            if (found) {
                position = newPosition;
                return true;
            }
        }
    }

    return false;
}

}

}
