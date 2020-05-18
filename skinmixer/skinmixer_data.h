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

    struct Entry {
        Model* model;
        std::vector<float> vertexFuzzyValue;
        std::vector<float> jointFuzzyValue;
    };

    typedef nvl::Index Index;


    SkinMixerData();
    ~SkinMixerData();

    void addEntry(Model* model);
    void deleteEntry(Model* model);

    const Entry& entry(Model* model) const;
    Entry& entry(Model* model);

    void clear();


private:

    std::vector<Entry> vEntries;
    std::unordered_map<Model*, Index> vModelMap;

};

}

#include "skinmixer_data.cpp"

#endif // SKINMIXER_DATA_H
