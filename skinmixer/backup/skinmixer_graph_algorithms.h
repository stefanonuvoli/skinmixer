#ifndef SKINMIXER_GRAPH_ALGORITHMS_H
#define SKINMIXER_GRAPH_ALGORITHMS_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer/skinmixer_graph.h"

namespace skinmixer {

template<class Model>
bool findVertexBirthNode(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const typename Model::Mesh::VertexId& vertexId,
        nvl::Index& birthNode,
        typename Model::Mesh::VertexId& birthVertex);

template<class Model>
bool findJointBirthNode(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const typename Model::Skeleton::JointId& jointId,
        nvl::Index& birthNode,
        typename Model::Skeleton::JointId& birthJoint);

template<class Model>
bool findOriginalPosition(
        const SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Index& rootNodeId,
        const typename Model::Mesh::Point& position,
        typename Model::Mesh::Point& originalPosition);


template<class Model>
void applyTransformationToNode(
        SkinMixerNode<Model>& node,
        const nvl::Affine3d& transformation);

template<class Model>
void applyTransformationToNode(
        SkinMixerGraph<Model>& graph,
        const nvl::Index& nodeId,
        const nvl::Affine3d& transformation);

}

#include "skinmixer_graph_algorithms.cpp"

#endif // SKINMIXER_GRAPH_ALGORITHMS_H
