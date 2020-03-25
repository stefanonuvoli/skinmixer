#include "skinmixer_node.h"

template<class Model>
SkinMixerNode<Model>::SkinMixerNode()
{

}

template<class Model>
SkinMixerNode<Model>::SkinMixerNode(Model* model, OperationType operation)
{
    this->model = model;
    this->operation = operation;
}
