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

    entry.frame.setIdentity();

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
void SkinMixerData<Model>::removeAction(const SkinMixerData::Index &index)
{
    vActions[index].clear();
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
void SkinMixerData<Model>::Entry::clear()
{
    id = nvl::MAX_INDEX;

    model = nullptr;

    birth.clear();
    relatedActions.clear();

    blendingAnimations.clear();
    blendingAnimationWeights.clear();

    frame = Transformation::Identity();
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

template<class Model>
void SkinMixerData<Model>::Action::clear()
{
    operation = OperationType::NONE;
    entry1 = nvl::MAX_INDEX;
    entry2 = nvl::MAX_INDEX;
    joint1 = nvl::MAX_INDEX;
    joint2 = nvl::MAX_INDEX;

    select1.clear();
    select2.clear();

    void clear();
}

template<class Model>
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Entry& entry)
{
    SelectInfo newSelectInfo;
    newSelectInfo.vertex.resize(entry.model->mesh.nextVertexId(), 1.0);
    newSelectInfo.joint.resize(entry.model->skeleton.jointNumber(), 1.0);

    for (Index aId : entry.relatedActions) {
        Action& action = this->action(aId);

        if (action.entry1 == entry.id) {
            const SelectInfo& selectInfo = action.select1;
            for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                newSelectInfo.vertex[vId] = std::min(newSelectInfo.vertex[vId], selectInfo.vertex[vId]);
            }
            for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                newSelectInfo.joint[jId] = std::min(newSelectInfo.joint[jId], selectInfo.joint[jId]);
            }
        }
        else if (action.entry2 == entry.id) {
            const SelectInfo& selectInfo = action.select2;
            for (VertexId vId = 0; vId < entry.model->mesh.nextVertexId(); ++vId) {
                newSelectInfo.vertex[vId] = std::min(newSelectInfo.vertex[vId], selectInfo.vertex[vId]);
            }
            for (JointId jId = 0; jId < entry.model->skeleton.jointNumber(); ++jId) {
                newSelectInfo.joint[jId] = std::min(newSelectInfo.joint[jId], selectInfo.joint[jId]);
            }
        }
    }

    return newSelectInfo;
}

template<class Model>
typename SkinMixerData<Model>::SelectInfo SkinMixerData<Model>::computeGlobalSelectInfo(const Index& eId)
{
    const Entry& currentEntry = this->entry(eId);

    return computeGlobalSelectInfo(currentEntry);
}

template<class Model>
void SkinMixerData<Model>::clear()
{
    vEntries.clear();
    vModelMap.clear();
    vActions.clear();
}

template<class Model>
void SkinMixerData<Model>::clearActions()
{
    this->vActions.clear();
    for (Entry& entry : this->vEntries) {
        entry.relatedActions.clear();
    }
}

}
