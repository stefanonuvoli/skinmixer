#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
SkinMixerData<Model>::SkinMixerData()
{

}

template<class Model>
SkinMixerData<Model>::~SkinMixerData()
{
    this->clear();
}

template<class Model>
void SkinMixerData<Model>::addEntry(Model* model)
{
    Entry entry;

    entry.model = model;

    entry.vertexFuzzyValue.resize(model->mesh.nextVertexId(), 1.0);
    entry.jointFuzzyValue.resize(model->skeleton.jointNumber(), 1.0);

    vEntries.push_back(entry);
    vModelMap.insert(std::make_pair(model, vEntries.size() - 1));
}

template<class Model>
void SkinMixerData<Model>::deleteEntry(Model* model)
{
    vModelMap.erase(model);
}

template<class Model>
const std::vector<typename SkinMixerData<Model>::Entry>& SkinMixerData<Model>::entries() const
{
    return vEntries;
}

template<class Model>
std::vector<typename SkinMixerData<Model>::Entry>& SkinMixerData<Model>::entries()
{
    return vEntries;
}

template<class Model>
typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(Model* model)
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
const std::vector<typename SkinMixerData<Model>::Action>& SkinMixerData<Model>::actions() const
{
    return vActions;
}

template<class Model>
void SkinMixerData<Model>::addAction(const Action& action)
{
    vActions.push_back(action);
}

template<class Model>
const typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(Model* model) const
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
void SkinMixerData<Model>::clear()
{
    vEntries.clear();
    vModelMap.clear();
}

}
