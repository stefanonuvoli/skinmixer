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
    typedef typename nvl::Translation3d Translation;
    typedef typename nvl::Rotation3d Rotation;
    typedef typename nvl::Affine3d Affine;
    typedef typename nvl::DualQuaterniond DualQuaternion;

    struct SelectInfo {
        std::vector<double> vertex;
        std::vector<double> joint;
        std::vector<bool> keepDiscard;

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

        std::vector<Index> blendingAnimationModes;
        std::vector<Index> blendingAnimationIds;
        std::vector<std::vector<double>> blendingAnimationWeights;
        std::vector<double> blendingAnimationSpeeds;

        std::vector<Affine> rotations;
        std::vector<Translation> translations;

        Index lastOriginalAnimationId;

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

    std::vector<Index> relatedActions(const Index& entryId);
    std::vector<Index> relatedActions(const Entry& entry);

    typename SkinMixerData<Model>::SelectInfo computeGlobalSelectInfo(const Index& entryId);
    typename SkinMixerData<Model>::SelectInfo computeGlobalSelectInfo(const Entry& entry);

    void applyJointDeformation(const Index& entryId, const JointId& jId, const Rotation& rotation, const Translation& translation);
    void applyJointDeformation(Entry& entry, const JointId& jId, const Rotation& rotation, const Translation& translation);
    void resetJointDeformation(const Index& entryId);
    void resetJointDeformation(Entry& entry);
    std::vector<DualQuaternion> computeJointDeformation(const Index& entryId);
    std::vector<DualQuaternion> computeJointDeformation(Entry& entry);
    std::vector<DualQuaternion> computeDeformation(const Index& entryId);
    std::vector<DualQuaternion> computeDeformation(Entry& entry);
    std::vector<std::vector<DualQuaternion>> computeDeformations();
    void deformModel(const Index& entryId);
    void deformModel(Entry& entry);
    void deformModels();

    void removeNonStandardTransformationsFromModel(const Index& entryId);
    void removeNonStandardTransformationsFromModel(Entry& entry);
    void removeNonStandardTransformationsFromModels();

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
