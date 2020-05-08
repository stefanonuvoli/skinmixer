#include "skinmixer_node.h"

namespace skinmixer {

template<class Model>
OperationGraphNode<Model>::OperationGraphNode() :
    model(nullptr),
    operation(Operation::NONE),
    transformation(nvl::Affine3d::Identity())
{

}

}
