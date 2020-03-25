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
    typedef Graph::Node SkinMixerNode;
    typedef Graph::OperationType OperationType;

public:

    explicit SkinMixerManager(nvl::Canvas* canvas = nullptr, QWidget *parent = nullptr);
    ~SkinMixerManager();

    Model* loadModelFromFile(const std::string& filename);

    Index addModelFromNode(const Index& nodeId, const std::string& name);
    Index addModelFromNode(SkinMixerNode& node, const std::string& name);

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
    void on_detachingSmoothCheckBox_stateChanged(int arg1);
    void on_detachingDetachButton_clicked();

    void on_attachingMoveButton_clicked();

private:

    void doDetach();
    void updateDetachPreview();
    void clearDetachPreview();

    void moveModelInPosition();

    void updateView();

    void updateSkinningWeightVertexValues();

    void initialize();
    void connectSignals();

    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;

    Graph skinMixerGraph;

    std::vector<Model*> vModels;
    std::vector<ModelDrawer*> vModelDrawers;

    nvl::ModelDrawer<Model>* vSelectedModelDrawer;
    Index vSelectedJointId;

//    std::vector<int> vPreviewFaceSegmentation;
//    std::vector<int> vPreviewJointSegmentation;

    bool detachPreview;
    PolylineMesh detachPreviewMesh;
    PolylineMeshDrawer detachPreviewDrawer;

};

}

#endif // DETACHER_MANAGER_H
