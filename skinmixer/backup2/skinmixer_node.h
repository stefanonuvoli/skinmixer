#ifndef SKINMIXER_NODE_H
#define SKINMIXER_NODE_H

#include <nvl/nuvolib.h>

#include <nvl/math/transformations.h>

#include "skinmixer/skinmixer_operation.h"

#include <vector>

namespace skinmixer {

template<class Model>
struct OperationGraphNode
{
public:

    OperationGraphNode();

    Model* model;

    std::vector<nvl::Index> parents;
    std::vector<nvl::Index> children;

    std::vector<typename Model::Mesh::VertexId> birthVertex;
    std::vector<typename Model::Mesh::FaceId> birthFace;
    std::vector<typename Model::Skeleton::JointId> birthJoint;

    std::vector<nvl::Index> birthVertexParentNodeId;
    std::vector<nvl::Index> birthFaceParentNodeId;
    std::vector<nvl::Index> birthJointParentNodeId;

    Operation operation;

    typename Model::Skeleton::JointId vCutJointId;
    std::vector<typename Model::Mesh::VertexId> vCutVertices;

    nvl::Affine3d transformation;
};

}

#include "skinmixer_node.cpp"

#endif // SKINMIXER_NODE_H
