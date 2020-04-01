#ifndef SKINMIXER_GRAPH_H
#define SKINMIXER_GRAPH_H

#include <nvl/nuvolib.h>

#include <vector>
#include <unordered_map>

#include "skinmixer/skinmixer_node.h"

namespace skinmixer {

template<class Model>
class SkinMixerGraph
{

public:

    typedef nvl::Index Index;
    typedef SkinMixerNode<Model> Node;
    typedef typename Node::OperationType OperationType;

    SkinMixerGraph();
    ~SkinMixerGraph();

    Index addNode(const Node& node);
    Index addNode(Model& model, OperationType operation = OperationType::NONE);

    Node& node(Index id);
    const Node& node(Index id) const;
    Node& node(Model* model);
    const Node& node(Model* model) const;

    Index nodeId(Model* model) const;
    Index nodeId(const Node& node) const;

    void clear();


private:

    std::vector<Node> vNodes;
    std::unordered_map<Model*, Index> vModelMap;

};

}

#include "skinmixer_graph.cpp"

#endif // SKINMIXER_GRAPH_H
