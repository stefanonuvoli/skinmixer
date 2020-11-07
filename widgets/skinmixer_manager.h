#ifndef SKINMIXER_MANAGER_H
#define SKINMIXER_MANAGER_H

#include <QFrame>
#include <QWidget>

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <nvl/viewer/widgets/canvas.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/skeletonjoint_list_widget.h>
#include <nvl/viewer/widgets/model_animation_widget.h>
#include <nvl/viewer/drawables/model_drawer.h>

#include <nvl/models/model.h>
#include <nvl/models/meshes.h>

#include "skinmixer/skinmixer_operation.h"
#include "skinmixer/skinmixer_data.h"

namespace Ui {
class SkinMixerManager;
}

class SkinMixerManager : public QFrame
{
    Q_OBJECT

    typedef nvl::Index Index;
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;
    typedef Model::Skeleton::JointId JointId;

    typedef skinmixer::OperationType OperationType;
    typedef skinmixer::SkinMixerData<Model> SkinMixerData;
    typedef typename SkinMixerData::Entry SkinMixerEntry;
    typedef typename SkinMixerData::BirthInfo::JointInfo JointInfo;
    typedef typename SkinMixerData::SelectInfo SelectInfo;

    typedef nvl::Canvas::PickingData PickingData;


public:

    explicit SkinMixerManager(
            nvl::Canvas* canvas,
            nvl::DrawableListWidget* drawableListWidget,
            nvl::SkeletonJointListWidget* skeletonJointListWidget,
            nvl::ModelAnimationWidget* modelAnimationWidget,
            QWidget *parent = nullptr);
    ~SkinMixerManager();

    Index loadModelFromFile(const std::string& filename);
    Index loadModel(Model* model);
    Index loadModel(const Model& model);
    bool removeModelDrawer(ModelDrawer* modelDrawer);

public Q_SLOTS:

    void slot_canvasPicking(const std::vector<PickingData>& data);
    void slot_jointSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedJoints);
    void slot_drawableSelectionChanged(const std::unordered_set<Index>& selectDrawables);
    void slot_movableFrameChanged();    
    void slot_drawableAdded(const Index& id, nvl::Drawable* drawable);


private slots:

    void on_modelLoadButton_clicked();
    void on_modelRemoveButton_clicked();
    void on_modelSaveButton_clicked();    
    void on_modelDuplicateButton_clicked();

    void on_functionSmoothingSlider_valueChanged(int value);
    void on_rigiditySlider_valueChanged(int value);

    void on_offset1Slider_valueChanged(int value);
    void on_offset2Slider_valueChanged(int value);

    void on_operationDetachButton_clicked();
    void on_operationRemoveButton_clicked();
    void on_operationAttachButton_clicked();
    void on_operationAbortButton_clicked();
    void on_operationApplyButton_clicked();

    void on_mixButton_clicked();
    void on_animationJointAllCheckBox_stateChanged(int arg1);
    void on_animationJointMeshComboBox_currentIndexChanged(int index);
    void on_animationBlendButton_clicked();
    void on_animationConfirmButton_clicked();
    void on_animationAbortButton_clicked();

    void on_updateValuesResetButton_clicked();
    void on_updateValuesWeightsButton_clicked();
    void on_updateValuesBirthButton_clicked();

private:

    ModelDrawer* getSelectedModelDrawer();
    JointId getSelectedJointId();

    void mix();
    void blendAnimations();

    void applyOperation();
    void abortOperation();    

    void updateCanvasPreview();
    void prepareModelForAttach();
    void updateView();

    void colorizeByData(
            ModelDrawer* modelDrawer);
    void colorizeBySelectValues(
            ModelDrawer* modelDrawer,
            const std::vector<double>& vertexSelectValue,
            const std::vector<double>& jointSelectValue);
    void colorizeByAnimationWeights(
            ModelDrawer* modelDrawer,
            const std::vector<std::vector<double>>& blendingAnimationWeights);

    void updateValuesReset();
    void updateValuesSkinningWeights();
    void updateValuesBirth();

    void initialize();
    void connectSignals();

    void clear();
    void initializeLoadedModel(Model* model);

    void clearLayout(QLayout *layout);

    //Fields
    std::unordered_set<Model*> vModels;
    std::unordered_map<ModelDrawer*, Model*> vDrawerToModelMap;
    std::unordered_map<Model*, ModelDrawer*> vModelToDrawerMap;

    //Interface fields
    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;
    nvl::DrawableListWidget* vDrawableListWidget;
    nvl::SkeletonJointListWidget* vSkeletonJointListWidget;
    nvl::ModelAnimationWidget* vModelAnimationWidget;

    OperationType vCurrentOperation;
    SkinMixerData vSkinMixerData;

    ModelDrawer* vSelectedModelDrawer;
    JointId vSelectedJoint;
    ModelDrawer* vAttachModelDrawer;
    JointId vAttachJoint;
    bool vPreparedAttach;
    nvl::Affine3d vBackupFrame;

    Index vBlendingAnimation;
    std::vector<QSlider*> animationWeightSliders;
    std::vector<QLabel*> animationWeightLabels;

};

#endif // SKINMIXER_MANAGER_H
