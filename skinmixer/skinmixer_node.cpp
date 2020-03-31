#include "skinmixer_node.h"

template<class Model>
SkinMixerNode<Model>::SkinMixerNode() :
    model(nullptr),
    operation(NONE),
    transformation(nvl::Affine3d::Identity())
{

}
