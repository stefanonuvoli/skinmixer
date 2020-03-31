#include "skinmixer_graph.h"


namespace skinmixer {

template<class Model>
SkinMixerGraph<Model>::SkinMixerGraph()
{

}

template<class Model>
SkinMixerGraph<Model>::~SkinMixerGraph()
{
    this->clear();
}

template<class Model>
typename SkinMixerGraph<Model>::Index SkinMixerGraph<Model>::addNode(const Node& node)
{
    Index id = vNodes.size();

    this->vNodes.push_back(node);
    this->vModelMap.insert(std::make_pair(node.model, id));

    return id;
}

template<class Model>
typename SkinMixerGraph<Model>::Index SkinMixerGraph<Model>::addNode(Model& model, OperationType operation)
{
    Node node;
    node.model = new Model(model);
    node.operation = operation;
    return this->addNode(node);
}

template<class Model>
typename SkinMixerGraph<Model>::Node& SkinMixerGraph<Model>::node(Index id)
{
    return vNodes[id];
}

template<class Model>
const typename SkinMixerGraph<Model>::Node& SkinMixerGraph<Model>::node(Index id) const
{
    return vNodes[id];
}

template<class Model>
typename SkinMixerGraph<Model>::Node& SkinMixerGraph<Model>::node(Model* model)
{
    return this->node(getIdByModel(model));
}

template<class Model>
const typename SkinMixerGraph<Model>::Node& SkinMixerGraph<Model>::node(Model* model) const
{
    return this->node(getIdByModel(model));
}

template<class Model>
const typename SkinMixerGraph<Model>::Index& SkinMixerGraph<Model>::getIdByModel(Model *model) const
{
    return vModelMap.at(model);
}

template<class Model>
void SkinMixerGraph<Model>::clear()
{
    for (Node& node : vNodes) {
        delete node.model;
        node.model = nullptr;
    }
    vNodes.clear();
    vModelMap.clear();
}

}
