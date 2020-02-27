#ifndef DETACHER_MANAGER_H
#define DETACHER_MANAGER_H

#include <QFrame>

#include <nvl/viewer/widgets/canvas.h>
#include <nvl/viewer/drawables/model_drawer.h>
#include <nvl/models/model.h>

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


public Q_SLOTS:

    void slot_selectedDrawableUpdated();
    void slot_drawableSelectionChanged();


private slots:

    void on_skeletonSegmentationPreviewButton_clicked();
    void on_skeletonSegmentationClearButton_clicked();
    void on_skeletonSegmentationCompactnessSpinBox_valueChanged(double arg1);
    void on_skeletonSegmentationDetachButton_clicked();

private:

    void detachBySegmentation();

    void updateView();

    void segmentationPreview();
    void clearSegmentationPreview();
    void showModelSegmentationColor();
    void resetModelSegmentationColor();

    void initialize();
    void connectSignals();

    Ui::SkinMixerManager *ui;

    nvl::Canvas* vCanvas;

    std::vector<nvl::Model3d*> models;
    std::vector<nvl::ModelDrawer<nvl::Model3d>*> modelDrawers;

    nvl::ModelDrawer<nvl::Model3d>* vSelectedModelDrawer;
    nvl::Index vSelectedJointId;

    std::vector<int> vPreviewSegmentation;

};

}

#endif // DETACHER_MANAGER_H
