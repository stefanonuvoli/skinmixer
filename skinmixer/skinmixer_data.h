#ifndef SKINMIXER_DATA_H
#define SKINMIXER_DATA_H

#include <nvl/nuvolib.h>

#include <vector>
#include <unordered_map>

#include "skinmixer/skinmixer_operation.h"

namespace skinmixer {

template<class Model>
class SkinMixerData
{

public:

    typedef typename Model::Skeleton::JointId JointId;

    struct Entry {
        Model* model;
        std::vector<float> vertexFuzzyValue;
        std::vector<float> jointFuzzyValue;
    };    

    struct Action {
        OperationType operation;
        Model* model1;
        Model* model2;
        JointId joint1;
        JointId joint2;
    };

    typedef nvl::Index Index;


    SkinMixerData();
    ~SkinMixerData();

    void addEntry(Model* model);
    void deleteEntry(Model* model);    

    const std::vector<Entry>& entries() const;
    std::vector<Entry>& entries();
    const Entry& entry(Model* model) const;
    Entry& entry(Model* model);

    const std::vector<Action>& actions() const;
    void addAction(const Action& action);

    void clear();


private:

    std::vector<Entry> vEntries;
    std::unordered_map<Model*, Index> vModelMap;

    std::vector<Action> vActions;

};

}

#include "skinmixer_data.cpp"

#endif // SKINMIXER_DATA_H
