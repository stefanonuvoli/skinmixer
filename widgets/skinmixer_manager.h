#ifndef DETACHER_MANAGER_H
#define DETACHER_MANAGER_H

#include <QFrame>

#include <nvl/viewer/widgets/canvas.h>
#include <nvl/viewer/drawables/model_drawer.h>
#include <nvl/models/model.h>
#include <nvl/models/meshes.h>

#include "skinmixer/skinmixer_graph.h"

namespace Ui {
class SkinMixerManager;
}

namespace skinmixer {

class SkinMixerManager : public QFrame
{
    Q_OBJECT

    typedef nvl::Index Index;
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;
    typedef nvl::PolylineMesh3d PolylineMesh;
    typedef nvl::PolylineMeshDrawer<PolylineMesh> PolylineMeshDrawer;
    typedef SkinMixerGraph<Model> Graph;
    typedef typename Graph::Node SkinMixerNode;
    typedef typename Graph::OperationType OperationType;

public:

    explicit SkinMixerManager(nvl::Canvas* canvas = nullptr, QWidget *parent = nullptr);
    ~SkinMixerManager();

    void loadModelFromFile(const std::string& filename);

    Index addModelDrawerFromNode(const Index& nodeId, const std::string& name);
    Index addModelDrawerFromNode(SkinMixerNode& node, const std::string& name);

public Q_SLOTS:

    void slot_drawableSelectionChanged();
    void slot_selectedDrawableUpdated();
    void slot_movableFrameChanged();


private slots:

    void on_modelsRemoveButton_clicked();
    void on_modelsLoadButton_clicked();

    void on_detachingPreviewButton_clicked();
    void on_detachingClearButton_clicked();
    void on_detachingOffsetSpinBox_valueChanged(double arg1);
    void on_detachingRigiditySpinBox_valueChanged(double arg1);
    void on_detachingSmoothCheckBox_stateChanged(int arg1);
    void on_detachingDetachButton_clicked();

    void on_transformationMoveButton_clicked();

    void on_attachingSelect1Button_clicked();
    void on_attachingSelect2Button_clicked();
    void on_attachingAbortButton_clicked();
    void on_attachingFindPositionButton_clicked();
    void on_attachingAttachButton_clicked();

private:

    void doDetach();
    void updateDetachPreview();
    void clearDetachPreview();

    void clearAttachSelection();
    void attachFindPosition();
    void doAttach();

    void nodeApplyFrameTransformation();

    void updateView();

    void updateSkinningWeightVertexValues();

    void initialize();
    void connectSignals();

    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;

    Graph vSkinMixerGraph;

    std::vector<Model*> vModels;
    std::vector<ModelDrawer*> vModelDrawers;

    nvl::ModelDrawer<Model>* vSelectedModelDrawer;
    Index vSelectedJointId;

    bool vDetachPreview;
    PolylineMesh vDetachPreviewMesh;
    PolylineMeshDrawer vDetachPreviewDrawer;

    nvl::ModelDrawer<Model>* vAttachSelectedModelDrawer1;
    Index vAttachSelectedJoint1;
    nvl::ModelDrawer<Model>* vAttachSelectedModelDrawer2;
    Index vAttachSelectedJoint2;

};

}

#endif // DETACHER_MANAGER_H
