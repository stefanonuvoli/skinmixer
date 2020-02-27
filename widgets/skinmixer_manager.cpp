#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/colorize.h>

#include "algorithms/skeleton_segmentation.h"
#include "algorithms/detach.h"

namespace skinmixer {

SkinMixerManager::SkinMixerManager(nvl::Canvas* canvas, QWidget *parent) :
    QFrame(parent),
    ui(new Ui::SkinMixerManager),
    vCanvas(canvas),
    vSelectedModelDrawer(nullptr),
    vSelectedJointId(nvl::MAX_ID)
{
    ui->setupUi(this);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
    delete ui;
}

void SkinMixerManager::slot_drawableSelectionChanged()
{
    clearSegmentationPreview();

    vSelectedModelDrawer = nullptr;
    vSelectedJointId = nvl::MAX_ID;

    const std::unordered_set<nvl::Index>& selectedItems = vCanvas->selectedDrawables();
    if (selectedItems.size() == 1) {
        nvl::Index firstItem = *selectedItems.begin();
        nvl::Drawable* drawable = vCanvas->drawable(firstItem);

        nvl::ModelDrawer<nvl::Model3d>* modelDrawer = dynamic_cast<nvl::ModelDrawer<nvl::Model3d>*>(drawable);
        if (modelDrawer != nullptr) {
            vSelectedModelDrawer = modelDrawer;

            const std::vector<long long int>& selectionData = vSelectedModelDrawer->skeletonDrawer().selectionData();
            if (selectionData.size() == 1) {
                vSelectedJointId = static_cast<nvl::Index>(*selectionData.begin());
            }
        }
    }

    updateView();
}

void SkinMixerManager::slot_selectedDrawableUpdated()
{
    vSelectedJointId = nvl::MAX_ID;

    if (vSelectedModelDrawer != nullptr) {
        const std::vector<long long int>& selectionData = vSelectedModelDrawer->skeletonDrawer().selectionData();
        if (selectionData.size() == 1) {
            vSelectedJointId = static_cast<nvl::Index>(*selectionData.begin());
        }
    }

    if (!vPreviewSegmentation.empty())
        segmentationPreview();

    updateView();
}

void SkinMixerManager::detachBySegmentation()
{
    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    std::vector<std::vector<nvl::Model3d::Mesh::VertexId>> vertexMaps;
    std::vector<std::vector<nvl::Model3d::Mesh::FaceId>> faceMaps;
    std::vector<std::vector<nvl::Model3d::Skeleton::JointId>> jointMaps;

    const float compactness = ui->skeletonSegmentationCompactnessSpinBox->value();

    std::vector<nvl::Model3d> detachResult = skinmixer::detachBySkeletonSegmentation(*vSelectedModelDrawer->model(), vSelectedJointId, compactness, vertexMaps, faceMaps, jointMaps);

    for (nvl::Model3d result : detachResult) {
        nvl::Model3d* newModel = new nvl::Model3d(result);
        nvl::ModelDrawer<nvl::Model3d>* newModelDrawer = new nvl::ModelDrawer<nvl::Model3d>(newModel);
        models.push_back(newModel);
        modelDrawers.push_back(newModelDrawer);

        newModelDrawer->skeletonDrawer().setVisible(true);
        vCanvas->addDrawable(newModelDrawer, "Result");

        newModelDrawer->setAnimationTargetFPS(60);
        newModelDrawer->setAnimationSkinningMode(nvl::ModelDrawer<nvl::Model3d>::SkinningMode::SKINNING_DUAL_QUATERNIONS);
        newModelDrawer->setAnimationLoop(true);
        newModelDrawer->setAnimationSpeed(1);
        newModelDrawer->loadAnimation(0);
    }

    clearSegmentationPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::updateView()
{
    bool itemSelected = vSelectedModelDrawer != nullptr && vSelectedJointId != nvl::MAX_ID;
    bool preview = !vPreviewSegmentation.empty();
    ui->skeletonSegmentationPreviewButton->setEnabled(itemSelected && !preview);
    ui->skeletonSegmentationClearButton->setEnabled(preview);
    ui->skeletonSegmentationDetachButton->setEnabled(itemSelected);
}

void SkinMixerManager::segmentationPreview()
{
    clearSegmentationPreview();

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    const float compactness = ui->skeletonSegmentationCompactnessSpinBox->value();
    vPreviewSegmentation = skinmixer::skeletonBinarySegmentationGraphcut(*vSelectedModelDrawer->model(), compactness, vSelectedJointId);

    showModelSegmentationColor();
    updateView();
}

void SkinMixerManager::clearSegmentationPreview()
{
    vPreviewSegmentation.clear();
    resetModelSegmentationColor();
}

void SkinMixerManager::showModelSegmentationColor()
{
    resetModelSegmentationColor();

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID || vPreviewSegmentation.empty()) {
        return;
    }

    nvl::Model3d* model = vSelectedModelDrawer->model();

    std::vector<nvl::Color> colors = nvl::getLabelDifferentColor(vPreviewSegmentation);

    int maxLabel = -1;
    for (const int& l : vPreviewSegmentation) {
        maxLabel = std::max(maxLabel, l);
    }

    for (const auto& face : model->mesh.faces()) {
        vSelectedModelDrawer->meshDrawer().setRenderingFaceColor(face.id(), colors[face.id()]);
    }

    vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vSelectedJointId, nvl::Color(50,50,50));

    vCanvas->updateGL();

    updateView();
}

void SkinMixerManager::resetModelSegmentationColor()
{
    if (vSelectedModelDrawer == nullptr)
        return;

    vSelectedModelDrawer->meshDrawer().resetRenderingFaceColors();
    vSelectedModelDrawer->skeletonDrawer().resetRenderingJointColors();

    vCanvas->updateGL();

    updateView();
}

void SkinMixerManager::initialize()
{
    updateView();
}

void SkinMixerManager::connectSignals()
{
    if (vCanvas != nullptr) {
        //Connect signals to the viewer
        connect(vCanvas, &nvl::Canvas::signal_drawableSelectionChanged, this, &SkinMixerManager::slot_drawableSelectionChanged);
        connect(vCanvas, &nvl::Canvas::signal_selectedDrawableUpdated, this, &SkinMixerManager::slot_selectedDrawableUpdated);
    }
}

void SkinMixerManager::on_skeletonSegmentationPreviewButton_clicked()
{
    segmentationPreview();
}

void SkinMixerManager::on_skeletonSegmentationClearButton_clicked()
{
    clearSegmentationPreview();
}

void SkinMixerManager::on_skeletonSegmentationCompactnessSpinBox_valueChanged(double arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (!vPreviewSegmentation.empty()) {
        segmentationPreview();
    }
}

void SkinMixerManager::on_skeletonSegmentationDetachButton_clicked()
{
    detachBySegmentation();
}

}
