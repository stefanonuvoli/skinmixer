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
    Index loadModel(Model* model, const std::string& name);
    Index loadModel(const Model& model, const std::string& name);
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

    void on_cutOffsetSlider_valueChanged(int value);    
    void on_cutOffsetResetButton_clicked();
    void on_cutRigiditySlider_valueChanged(int value);
    void on_cutOffsetRigidityButton_clicked();

    void on_operationDetachButton_clicked();
    void on_operationRemoveButton_clicked();
    void on_operationAttachButton_clicked();
    void on_operationAbortButton_clicked();
    void on_operationApplyButton_clicked();

    void on_mixButton_clicked();    
    void on_blendAnimationsButton_clicked();

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

    void updateCanvasView();
    void prepareModelForAttach();
    void updateView();

    void colorizeModelDrawerWithSelectValues(
            ModelDrawer* modelDrawer);
    void colorizeModelDrawerWithSelectValues(
            ModelDrawer* modelDrawer,
            const std::vector<double>& vertexSelectValue,
            const std::vector<bool>& jointSelectValue);

    void updateValuesReset();
    void updateValuesSkinningWeights();
    void updateValuesBirth();

    void initialize();
    void connectSignals();

    void clear();
    void initializeLoadedModel(Model* model);

    //Fields
    std::unordered_set<Model*> vModels;
    std::unordered_set<ModelDrawer*> vModelDrawers;
    std::unordered_map<ModelDrawer*, Model*> vModelMap;

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
    nvl::Affine3d vBackupFrame;

};

#endif // SKINMIXER_MANAGER_H
