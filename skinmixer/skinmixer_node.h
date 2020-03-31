#ifndef SKINMIXER_NODE_H
#define SKINMIXER_NODE_H

#include <nvl/nuvolib.h>

#include <nvl/math/transformations.h>

#include <vector>

template<class Model>
struct SkinMixerNode
{
public:

    enum OperationType { NONE, DETACH, ATTACH };

    SkinMixerNode();

    Model* model;    
    OperationType operation;

    std::vector<nvl::Index> parents;
    std::vector<nvl::Index> children;

    std::vector<typename Model::Mesh::VertexId> birthVertex;
    std::vector<typename Model::Mesh::FaceId> birthFace;
    std::vector<typename Model::Skeleton::JointId> birthJoint;

    std::vector<nvl::Index> birthVertexParentNodeId;
    std::vector<nvl::Index> birthFaceParentNodeId;
    std::vector<nvl::Index> birthJointParentNodeId;

    typename Model::Skeleton::JointId detachingJointId;

    nvl::Affine3d transformation;
};

#include "skinmixer_node.cpp"

#endif // SKINMIXER_NODE_H
