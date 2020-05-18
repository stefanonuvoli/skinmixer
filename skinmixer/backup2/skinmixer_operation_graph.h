#ifndef SKINMIXER_OPERATION_GRAPH_H
#define SKINMIXER_OPERATION_GRAPH_H

#include <nvl/nuvolib.h>

#include <vector>
#include <unordered_map>

#include "skinmixer/skinmixer_node.h"
#include "skinmixer/skinmixer_operation.h"

namespace skinmixer {

template<class Model>
class OperationGraph
{

public:

    typedef nvl::Index Index;
    typedef OperationGraphNode<Model> Node;

    OperationGraph();
    ~OperationGraph();

    Index addNode(const Node& node);
    Index addNode(Model* model, Operation operation = Operation::NONE);

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

#include "skinmixer_operation_graph.cpp"

#endif // SKINMIXER_OPERATION_GRAPH_H
