#ifndef DETACHER_MANAGER_H
#define DETACHER_MANAGER_H

#include <QFrame>
#include <QWidget>

#include <nvl/viewer/widgets/canvas.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/skeletonjoint_list_widget.h>
#include <nvl/viewer/widgets/model_loader_widget.h>
#include <nvl/viewer/drawables/model_drawer.h>

#include <nvl/models/model.h>
#include <nvl/models/meshes.h>

#include "skinmixer/skinmixer_operation_graph.h"

namespace Ui {
class SkinMixerManager;
}

class SkinMixerManager : public QFrame
{
    Q_OBJECT

    typedef nvl::Index Index;
    typedef nvl::Model3d Model;
    typedef nvl::PolylineMesh3d PolylineMesh;
    typedef nvl::ModelDrawer<Model> ModelDrawer;
    typedef nvl::PolylineMeshDrawer<PolylineMesh> PolylineMeshDrawer;
    typedef Model::Skeleton::JointId JointId;

    typedef skinmixer::OperationGraph<Model> OperationGraph;
    typedef typename OperationGraph::Node OperationGraphNode;
    typedef skinmixer::Operation Operation;

    typedef nvl::Canvas::PickingData PickingData;

public:

    explicit SkinMixerManager(
            nvl::Canvas* canvas,
            nvl::DrawableListWidget* drawableListWidget,
            nvl::SkeletonJointListWidget* skeletonJointListWidget,
            nvl::ModelLoaderWidget* modelLoaderWidget,
            QWidget *parent = nullptr);
    ~SkinMixerManager();


public Q_SLOTS:

    void slot_canvasPicking(const std::vector<PickingData>& data);
    void slot_jointSelectionChanged(const std::unordered_set<Index>& selectedJoints);
    void slot_drawableSelectionChanged(const std::unordered_set<Index>& selectDrawables);
    void slot_movableFrameChanged();    
    void slot_drawableAdded(const Index& id, nvl::Drawable* drawable);


private slots:

    void on_operationRemoveRadio_toggled(bool checked);
    void on_operationDetachRadio_toggled(bool checked);
    void on_operationSplitRadio_toggled(bool checked);
    void on_operationAddRadio_toggled(bool checked);
    void on_operationReplaceRadio_toggled(bool checked);

    void on_operationApplyButton_clicked();
    void on_operationPreviewCheckBox_stateChanged(int arg1);

    void on_cutOffsetSlider_valueChanged(int value);
    void on_cutRigiditySlider_valueChanged(int value);
    void on_cutSmoothCheckBox_stateChanged(int arg1);


    void on_cutOffsetRigidityButton_clicked();

    void on_cutOffsetResetButton_clicked();

private:

    void doRemove();
    void doDetach();
    void doSplit();
    void doCutOperation(const Operation& operation);


    ModelDrawer* getSelectedModelDrawer();
    JointId getSelectedJointId();


    void updateView();
    void updatePreview();
    void updateSkinningWeightVertexValues();
    void updateSelectionColorization();

    void initialize();
    void connectSignals();

    //Fields
    OperationGraph vOperationGraph;
    std::unordered_map<Model*, Index> vNodeMap;
    std::vector<Model*> vModels;
    std::vector<ModelDrawer*> vModelDrawers;

    //Interface fields
    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;
    nvl::DrawableListWidget* vDrawableListWidget;
    nvl::SkeletonJointListWidget* vSkeletonJointListWidget;
    nvl::ModelLoaderWidget* vModelLoaderWidget;

    PolylineMesh vCutPreviewMesh;
    PolylineMeshDrawer vCutPreviewDrawer;

    ModelDrawer* lastSelectedModelDrawer;

};

#endif // DETACHER_MANAGER_H
