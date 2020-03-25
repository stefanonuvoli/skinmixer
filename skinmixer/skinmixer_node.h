#ifndef SKINMIXER_NODE_H
#define SKINMIXER_NODE_H

#include <nvl/nuvolib.h>

#include <vector>

template<class Model>
class SkinMixerNode
{

public:

    enum OperationType { NONE, DETACH, ATTACH };

    SkinMixerNode();
    SkinMixerNode(Model* model, OperationType operation);

    Model* model;

    std::vector<nvl::Index> parents;
    std::vector<nvl::Index> children;

    OperationType operation;

    std::vector<nvl::Index> birthVertexParent;
    std::vector<nvl::Index> birthFaceParent;
    std::vector<nvl::Index> birthJointParent;
    std::vector<typename Model::Mesh::VertexId> birthVertex;
    std::vector<typename Model::Mesh::FaceId> birthFace;
    std::vector<typename Model::Skeleton::JointId> birthJoint;

};

#include "skinmixer_node.cpp"

#endif // SKINMIXER_NODE_H
