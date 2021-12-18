#ifndef SKINMIXER_DATA_H
#define SKINMIXER_DATA_H

#include <nvl/nuvolib.h>

#include <vector>
#include <unordered_map>

#include <nvl/math/affine.h>
#include <nvl/math/dual_quaternion.h>

namespace skinmixer {

enum OperationType { NONE, REMOVE, DETACH, REPLACE, ATTACH };
enum ReplaceMode { BLEND, UNION };
enum MixMode { MESHING, MORPHING, PREVIEW };

template<class Model>
class SkinMixerData
{

public:

    typedef typename Model::Skeleton::JointId JointId;
    typedef typename Model::Mesh::VertexId VertexId;
    typedef typename Model::Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;
    typedef typename nvl::Affine3d Transformation;

    struct SelectInfo {
        std::vector<double> vertex;
        std::vector<double> joint;

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
        std::vector<JointId> mergeJoints;

        void clear();
    };

    struct Entry {
        Index id;

        Model* model;

        BirthInfo birth;

        std::vector<Index> relatedActions;

        std::vector<Index> blendingAnimationModes;
        std::vector<Index> blendingAnimationIds;
        std::vector<std::vector<double>> blendingAnimationWeights;
        std::vector<double> blendingAnimationSpeeds;

        std::vector<nvl::DualQuaterniond> deformation;


        void clear();
    };    

    struct Action {
        OperationType operation;
        Index entry1;
        Index entry2;
        JointId joint1;
        JointId joint2;

        SelectInfo select1;
        SelectInfo select2;
        double hardness1;
        double hardness2;

        nvl::Affine3d rotation2;
        nvl::Translation3d translation2;

        ReplaceMode replaceMode;

        void clear();
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
    void removeAction(const Index& index);
    nvl::Size actionNumber() const;
    const std::vector<Action>& actions() const;
    const Action& action(const Index& index) const;
    Action& action(const Index& index);    

    typename SkinMixerData<Model>::SelectInfo computeGlobalSelectInfo(const Entry& entry);
    typename SkinMixerData<Model>::SelectInfo computeGlobalSelectInfo(const Index& eId);

    void computeDeformation(const Index& entryId);
    void computeDeformation(Entry& entry);

    void clear();
    void clearActions();


private:

    std::vector<Entry> vEntries;
    std::unordered_map<const Model*, Index> vModelMap;

    std::vector<Action> vActions;

};

}

#include "skinmixer_data.cpp"

#endif // SKINMIXER_DATA_H
