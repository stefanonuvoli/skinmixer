#include "skinmixer_graph_algorithms.h"

namespace skinmixer {

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

}
