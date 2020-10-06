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
    typedef typename Model::Mesh::VertexId VertexId;
    typedef typename Model::Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;
    typedef typename nvl::Scaling3d ScalingTransform;

    struct SelectInfo {
        std::vector<double> vertex;
        std::vector<bool> joint;

        void clear();
    };

    struct BirthInfo {
        struct VertexInfo {
            Index eId;

            VertexId vId;

            FaceId closestFaceId;
            double distance;

            double weight;
        };
        struct JointInfo {
            Index eId;

            JointId jId;

            double confidence;
        };

        std::vector<Index> entries;
        std::vector<std::vector<VertexInfo>> vertex;
        std::vector<std::vector<JointInfo>> joint;

        void clear();
    };

    struct Entry {
        Index id;

        Model* model;

        SelectInfo select;
        BirthInfo birth;

        std::vector<std::vector<double>> animationWeights;

        std::vector<Index> relatedActions;

        void clear();
    };    

    struct Action {
        OperationType operation;
        Index entry1;
        Index entry2;
        JointId joint1;
        JointId joint2;
    };


    SkinMixerData();
    ~SkinMixerData();

    nvl::Index addEntry(Model* model);
    void deleteEntry(Model* model);    

    nvl::Size entryNumber() const;
    const std::vector<Entry>& entries() const;
    std::vector<Entry>& entries();
    const Entry& entry(const Index& index) const;
    Entry& entry(const Index& index);
    const Entry& entryFromModel(const Model* model) const;
    Entry& entryFromModel(const Model* model);

    nvl::Index addAction(const Action& action);
    nvl::Size actionNumber() const;
    const std::vector<Action>& actions() const;
    const Action& action(const Index& index) const;
    Action& action(const Index& index);

    void clear();


private:

    std::vector<Entry> vEntries;
    std::unordered_map<Model*, Index> vModelMap;

    std::vector<Action> vActions;

};

}

#include "skinmixer_data.cpp"

#endif // SKINMIXER_DATA_H
