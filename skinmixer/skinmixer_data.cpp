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
nvl::Index SkinMixerData<Model>::addEntry(Model* model)
{
    Entry entry;

    entry.id = vEntries.size();

    entry.model = model;

    vEntries.push_back(entry);
    vModelMap.insert(std::make_pair(model, entry.id));

    return entry.id;
}

template<class Model>
void SkinMixerData<Model>::deleteEntry(Model* model)
{
    typename std::unordered_map<Model*, Index>::iterator it = vModelMap.find(model);
    if (it != vModelMap.end()) {
        it->second->clear();
        vModelMap.erase(model);
    }
}

template<class Model>
nvl::Size SkinMixerData<Model>::entryNumber() const
{
    return vEntries.size();
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
typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(const Index& id)
{
    return vEntries.at(id);
}

template<class Model>
const typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entry(const Index& id) const
{
    return vEntries.at(id);
}

template<class Model>
typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entryFromModel(const Model* model)
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
const typename SkinMixerData<Model>::Entry& SkinMixerData<Model>::entryFromModel(const Model* model) const
{
    return vEntries.at(vModelMap.at(model));
}

template<class Model>
nvl::Index SkinMixerData<Model>::addAction(const Action& action)
{
    vActions.push_back(action);
    return vActions.size() - 1;
}

template<class Model>
nvl::Size SkinMixerData<Model>::actionNumber() const
{
    return vActions.size();
}

template<class Model>
const std::vector<typename SkinMixerData<Model>::Action>& SkinMixerData<Model>::actions() const
{
    return vActions;
}

template<class Model>
typename SkinMixerData<Model>::Action& SkinMixerData<Model>::action(const Index& id)
{
    return vActions.at(id);
}

template<class Model>
const typename SkinMixerData<Model>::Action& SkinMixerData<Model>::action(const Index& id) const
{
    return vActions.at(id);
}

template<class Model>
void SkinMixerData<Model>::clear()
{
    vEntries.clear();
    vModelMap.clear();
    vActions.clear();
}

template<class Model>
void SkinMixerData<Model>::Entry::clear()
{
    id = nvl::MAX_ID;

    model = nullptr;

    select.clear();
    birth.clear();
}

template<class Model>
void SkinMixerData<Model>::SelectInfo::clear()
{
    vertex.clear();
    joint.clear();
}

template<class Model>
void SkinMixerData<Model>::BirthInfo::clear()
{
    vertex.clear();
    joint.clear();
}

}
