#include "attach.h"

#include "skinmixer/skinmixer_graph_algorithms.h"

namespace skinmixer {

template<class Model>
nvl::Affine3d attachFindTransformation(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId1,
        const nvl::Index& nodeId2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2)
{
    typedef typename Model::Mesh::Point Point;
    typedef typename Model::Skeleton Skeleton;

    const Model* model1 = skinMixerGraph.node(nodeId1).model;
    const Skeleton& skeleton1 = model1->skeleton;
    const Model* model2 = skinMixerGraph.node(nodeId2).model;
    const Skeleton& skeleton2 = model2->skeleton;

    Point v1 = currentTransformation1 * (skeleton1.joint(targetJoint1).restTransform() * Point(0,0,0));
    Point v2 = currentTransformation2 * (skeleton2.joint(targetJoint2).restTransform() * Point(0,0,0));
    Point t = v1 - v2;

    return nvl::getTranslationAffine3(t);
}

template<class Model>
std::vector<nvl::Index> attach(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId1,
        const nvl::Index& nodeId2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2)
{
//    typedef SkinMixerNode<Model> Node;
//    typedef nvl::Index Index;
//    typedef typename Model::Mesh Mesh;
//    typedef typename Model::Skeleton Skeleton;
//    typedef typename Mesh::VertexId VertexId;
//    typedef typename Mesh::FaceId FaceId;
//    typedef typename Skeleton::JointId JointId;

    std::vector<nvl::Index> newNodes;

    //Apply transformation to the node
    skinmixer::applyTransformationToNode(skinMixerGraph.node(nodeId1), currentTransformation1);
    skinmixer::applyTransformationToNode(skinMixerGraph.node(nodeId2), currentTransformation2);


//    Mesh resultMesh;
//    std::vector<nvl::Segment<typename Model::Mesh::Point>> curveCoordinates;

//    std::vector<int> faceSegmentation;
//    std::vector<int> jointSegmentation;

//    std::vector<std::vector<VertexId>> birthVertex;
//    std::vector<std::vector<FaceId>> birthFace;
//    std::vector<std::vector<JointId>> birthJoint;
//    std::vector<Model> models = detachModel(
//                *(skinMixerGraph.node(nodeId).model),
//                targetJoint,
//                offset,
//                rigidity,
//                keepEntireSkeleton,
//                smooth,
//                curveCoordinates,
//                birthVertex,
//                birthFace,
//                birthJoint,
//                resultMesh,
//                faceSegmentation,
//                jointSegmentation);

//    for (Index i = 0; i < models.size(); ++i) {
//        const Model& model = models[i];
//        const Skeleton& skeleton = model.skeleton;

//        Node node;
//        node.model = new Model(model);
//        node.operation = Node::OperationType::DETACH;

//        node.parents.push_back(nodeId);

//        node.birthVertex = birthVertex[i];
//        node.birthFace = birthFace[i];
//        node.birthJoint = birthJoint[i];

//        node.birthVertexParentNodeId = std::vector<Index>(node.birthVertex.size(), 0);
//        node.birthFaceParentNodeId = std::vector<Index>(node.birthFace.size(), 0);
//        node.birthJointParentNodeId = std::vector<Index>(node.birthJoint.size(), 0);

//        node.vDetachingRootJointId = targetJoint;
//        for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
//            if (birthVertex[jId] == targetJoint) {
//                node.vDetachingJointId = jId;
//                break;
//            }
//        }

//        size_t newNodeId = skinMixerGraph.addNode(node);
//        newNodes.push_back(newNodeId);
//    }

//    Node& rootNode = skinMixerGraph.node(nodeId);
//    for (Index newNodeId : newNodes) {
//        blendSkinningWeights(skinMixerGraph, newNodeId);
//        rootNode.children.push_back(newNodeId);
//    }

   return newNodes;
}


}
