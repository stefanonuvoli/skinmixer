#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/colorize.h>
#include <nvl/io/model_io.h>
#include <nvl/models/mesh_normals.h>

#include "algorithms/skeleton_segmentation.h"
#include "algorithms/detach.h"

#include <QFileDialog>

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
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

    for (Model*& model : vModels) {
        if (model != nullptr) {
            delete model;
            model = nullptr;
        }
    }

    for (ModelDrawer*& drawers : vModelDrawers) {
        if (drawers != nullptr) {
            delete drawers;
            drawers = nullptr;
        }
    }

    delete ui;
}

void SkinMixerManager::slot_selectedDrawableUpdated()
{
    vSelectedJointId = nvl::MAX_ID;

    if (vSelectedModelDrawer != nullptr) {
        const std::vector<long long int>& selectionData = vSelectedModelDrawer->skeletonDrawer().selectionData();
        if (selectionData.size() == 1) {
            vSelectedJointId = static_cast<nvl::Index>(*selectionData.begin());
        }

        updateVertexValues();
    }

    if (!vPreviewSegmentation.empty())
        segmentationPreview();

    updateView();
}

void SkinMixerManager::slot_drawableSelectionChanged()
{
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

    clearSegmentationPreview();

    vSelectedModelDrawer = nullptr;
    vSelectedJointId = nvl::MAX_ID;

    const std::unordered_set<nvl::Index>& selectedItems = vCanvas->selectedDrawables();
    if (selectedItems.size() == 1) {
        nvl::Index firstItem = *selectedItems.begin();
        nvl::Drawable* drawable = vCanvas->drawable(firstItem);

        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(drawable);
        if (modelDrawer != nullptr) {
            vSelectedModelDrawer = modelDrawer;

            const std::vector<long long int>& selectionData = vSelectedModelDrawer->skeletonDrawer().selectionData();
            if (selectionData.size() == 1) {
                vSelectedJointId = static_cast<nvl::Index>(*selectionData.begin());
            }
        }

        updateVertexValues();
    }

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    updateView();
}

void SkinMixerManager::slot_movableFrameChanged()
{
    nvl::Quaterniond rot(vCanvas->movableFrame().rotation());
    nvl::Translation3d tra(vCanvas->movableFrame().translation());

    for (nvl::Index id : vCanvas->selectedDrawables()) {
        if (vCanvas->isFrameable(id)) {
            nvl::Frameable* frameable = vCanvas->frameable(id);
            const nvl::Affine3d& frame = frameable->frame();

            nvl::Translation3d lastTra(frame.translation());
            nvl::Quaterniond lastRot(frame.rotation());

            nvl::Affine3d newFrame = nvl::Affine3d::Identity();

            //Rotation
            newFrame = rot * lastRot * newFrame;

            //Translation
            newFrame = tra * lastTra * newFrame;

            frameable->setFrame(newFrame);
        }
    }

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());
}

void SkinMixerManager::detachBySegmentation()
{
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    std::vector<std::vector<Model::Mesh::VertexId>> vertexMaps;
    std::vector<std::vector<Model::Mesh::FaceId>> faceMaps;
    std::vector<std::vector<Model::Skeleton::JointId>> jointMaps;

    const float compactness = ui->detachingCompactnessSpinBox->value();

    std::vector<Model> detachResult = skinmixer::detachBySkeletonSegmentation(*vSelectedModelDrawer->model(), vSelectedJointId, compactness, vertexMaps, faceMaps, jointMaps);

    for (Model result : detachResult) {
        Model* newModel = new Model(result);
        ModelDrawer* newModelDrawer = new ModelDrawer(newModel);
        vModels.push_back(newModel);
        vModelDrawers.push_back(newModelDrawer);

        newModelDrawer->skeletonDrawer().setVisible(true);
        vCanvas->addDrawable(newModelDrawer, "Result");

        //TODO
        newModelDrawer->setAnimationTargetFPS(60);
        newModelDrawer->setAnimationSkinningMode(ModelDrawer::SkinningMode::SKINNING_DUAL_QUATERNIONS);
        newModelDrawer->setAnimationLoop(true);
        newModelDrawer->setAnimationSpeed(1);
        newModelDrawer->loadAnimation(0);
    }

    clearSegmentationPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::updateView()
{
    bool drawableSelected = vSelectedModelDrawer != nullptr;
    ui->modelsRemoveButton->setEnabled(drawableSelected);
    ui->modelsSaveButton->setEnabled(drawableSelected);

    bool jointSelected = drawableSelected && vSelectedJointId != nvl::MAX_ID;
    bool preview = !vPreviewSegmentation.empty();

    ui->detachingPreviewButton->setEnabled(jointSelected && !preview);
    ui->detachingClearButton->setEnabled(preview);
    ui->detachingDetachButton->setEnabled(jointSelected);
}

void SkinMixerManager::segmentationPreview()
{
    clearSegmentationPreview();

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    const float compactness = ui->detachingCompactnessSpinBox->value();
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
    typedef nvl::Model3d Model;

    resetModelSegmentationColor();

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID || vPreviewSegmentation.empty()) {
        return;
    }

    Model* model = vSelectedModelDrawer->model();

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

void SkinMixerManager::updateVertexValues()
{
    if (vSelectedModelDrawer != nullptr) {
        std::vector<double> vertexValues;

        if (vSelectedJointId != nvl::MAX_ID) {
            vertexValues.resize(vSelectedModelDrawer->model()->mesh.vertexNumber(), 0);
            for (auto vertex : vSelectedModelDrawer->model()->mesh.vertices()) {
                vertexValues[vertex.id()] = vSelectedModelDrawer->model()->skinningWeights.weight(vertex.id(), vSelectedJointId);
            }
        }
        vSelectedModelDrawer->meshDrawer().setVertexValues(vertexValues);

        vCanvas->updateGL();
    }
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
        connect(vCanvas, &nvl::Canvas::signal_movableFrameChanged, this, &SkinMixerManager::slot_movableFrameChanged);
    }
}

void SkinMixerManager::on_modelsLoadButton_clicked()
{
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter("Model (*.txt)");

    QStringList files;

    if (dialog.exec())
        files = dialog.selectedFiles();

    for (QString str : files) {

        nvl::FilenameInfo fileInfo = nvl::getFilenameInfo(str.toStdString());

        Model* model = new Model();

        nvl::modelLoadFromFile(str.toStdString(), *model);

        nvl::meshUpdateFaceNormals(model->mesh);
        nvl::meshUpdateVertexNormals(model->mesh);

        ModelDrawer* modelDrawer = new ModelDrawer(model);
        modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
        modelDrawer->meshDrawer().setWireframeVisible(true);
        modelDrawer->skeletonDrawer().setVisible(true);

        vModels.push_back(model);
        vModelDrawers.push_back(modelDrawer);

        vCanvas->addDrawable(modelDrawer, fileInfo.file);
    }

    vCanvas->fitScene();
}

void SkinMixerManager::on_modelsRemoveButton_clicked()
{
    const std::unordered_set<nvl::Index> selectedDrawables = vCanvas->selectedDrawables();
    for (nvl::Index id : selectedDrawables) {
        vCanvas->removeDrawable(id);
    }
    vCanvas->updateGL();
}

void SkinMixerManager::on_detachingPreviewButton_clicked()
{
    segmentationPreview();
}

void SkinMixerManager::on_detachingClearButton_clicked()
{
    clearSegmentationPreview();
}

void SkinMixerManager::on_detachingCompactnessSpinBox_valueChanged(double arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (!vPreviewSegmentation.empty()) {
        segmentationPreview();
    }
}

void SkinMixerManager::on_detachingDetachButton_clicked()
{
    detachBySegmentation();
}

}

