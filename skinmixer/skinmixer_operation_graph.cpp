#include "skinmixer_operation_graph.h"

namespace skinmixer {

template<class Model>
OperationGraph<Model>::OperationGraph()
{

}

template<class Model>
OperationGraph<Model>::~OperationGraph()
{
    this->clear();
}

template<class Model>
typename OperationGraph<Model>::Index OperationGraph<Model>::addNode(const Node& node)
{
    Index id = vNodes.size();

    this->vNodes.push_back(node);
    this->vModelMap.insert(std::make_pair(node.model, id));

    return id;
}

template<class Model>
typename OperationGraph<Model>::Index OperationGraph<Model>::addNode(Model* model, Operation operation)
{
    Node node;
    node.model = model;
    node.operation = operation;
    return this->addNode(node);
}

template<class Model>
typename OperationGraph<Model>::Node& OperationGraph<Model>::node(Index id)
{
    return vNodes[id];
}

template<class Model>
const typename OperationGraph<Model>::Node& OperationGraph<Model>::node(Index id) const
{
    return vNodes[id];
}

template<class Model>
typename OperationGraph<Model>::Node& OperationGraph<Model>::node(Model* model)
{
    return this->node(nodeId(model));
}

template<class Model>
const typename OperationGraph<Model>::Node& OperationGraph<Model>::node(Model* model) const
{
    return this->node(nodeId(model));
}

template<class Model>
typename OperationGraph<Model>::Index OperationGraph<Model>::nodeId(const Node& node) const
{
    return std::distance(vNodes.begin(), std::find(vNodes.begin(), vNodes.end(), node));
}

template<class Model>
typename OperationGraph<Model>::Index OperationGraph<Model>::nodeId(Model *model) const
{
    return vModelMap.at(model);
}

template<class Model>
void OperationGraph<Model>::clear()
{
    vNodes.clear();
    vModelMap.clear();
}

}
