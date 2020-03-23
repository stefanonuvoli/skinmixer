#ifndef DETACHER_MANAGER_H
#define DETACHER_MANAGER_H

#include <QFrame>

#include <nvl/viewer/widgets/canvas.h>
#include <nvl/viewer/drawables/model_drawer.h>
#include <nvl/models/model.h>
#include <nvl/models/meshes.h>

namespace Ui {
class SkinMixerManager;
}

namespace skinmixer {

class SkinMixerManager : public QFrame
{
    Q_OBJECT

public:

    explicit SkinMixerManager(nvl::Canvas* canvas = nullptr, QWidget *parent = nullptr);
    ~SkinMixerManager();

    void loadModelFromFile(const std::string& filename);

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

    void detachBySkeletonWeights();
    void updateDetachPreview();
    void clearDetachPreview();

    void moveModelInPosition();

    void updateView();

    void updateSkinningWeightVertexValues();

    void initialize();
    void connectSignals();

    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;

    std::vector<nvl::Model3d*> vModels;
    std::vector<nvl::ModelDrawer<nvl::Model3d>*> vModelDrawers;

    nvl::ModelDrawer<nvl::Model3d>* vSelectedModelDrawer;
    nvl::Index vSelectedJointId;

//    std::vector<int> vPreviewFaceSegmentation;
//    std::vector<int> vPreviewJointSegmentation;

    bool detachPreview;
    nvl::PolylineMesh3d detachPreviewMesh;
    nvl::PolylineMeshDrawer<nvl::PolylineMesh3d> detachPreviewDrawer;

};

}

#endif // DETACHER_MANAGER_H
