#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/colorize.h>

#include <nvl/io/model_io.h>

#include <nvl/models/mesh_normals.h>
#include <nvl/models/model_transformations.h>

#include "algorithms/skeleton_segmentation.h"
#include "algorithms/detach.h"

#include <QFileDialog>
#include <QMessageBox>

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

        updateSkinningWeightVertexValues();
    }

    if (!vPreviewFaceSegmentation.empty())
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

        updateSkinningWeightVertexValues();
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
            nvl::Drawable* drawable = vCanvas->drawable(id);

            const nvl::Affine3d& frame = frameable->frame();

            nvl::Translation3d lastTra(frame.translation());
            nvl::Quaterniond lastRot(frame.rotation());

            nvl::Point3d center = drawable->sceneCenter();
            nvl::Translation3d originTra(-center);

            nvl::Affine3d newFrame = frame;

            //Go to center
            newFrame = originTra * newFrame;

            //Rotation
            newFrame = rot * newFrame;

            //Back to initial position
            newFrame = originTra.inverse() * newFrame;

            //Translation
            newFrame = tra * newFrame;

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

    std::vector<Model> detachResult =
            skinmixer::detachBySkeletonSegmentation(
                *vSelectedModelDrawer->model(),
                vSelectedJointId,
                compactness,
                ui->detachingKeepSkeletonCheckBox->isChecked(),
                vertexMaps,
                faceMaps,
                jointMaps);

    for (Model result : detachResult) {
        Model* newModel = new Model(result);
        ModelDrawer* newModelDrawer = new ModelDrawer(newModel);

        vModels.push_back(newModel);
        vModelDrawers.push_back(newModelDrawer);

        vCanvas->addDrawable(newModelDrawer, "Result");
    }

    vSelectedModelDrawer->setVisible(false);
    vCanvas->notifySelectedDrawableUpdated();

    clearSegmentationPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::moveModelInPosition()
{
    typedef nvl::Model3d Model;
    if (vSelectedModelDrawer == nullptr) {
        return;
    }

    Model& model = *vSelectedModelDrawer->model();

    nvl::modelApplyTransformation(model, vSelectedModelDrawer->frame());
    nvl::meshUpdateFaceNormals(model.mesh);
    nvl::meshUpdateVertexNormals(model.mesh);

    vSelectedModelDrawer->update();
    vSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());

    vCanvas->notifySelectedDrawableUpdated();
    vCanvas->updateGL();
}

void SkinMixerManager::loadModelFromFile(const std::string& filename)
{
    if (filename.empty())
        return;

    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

    Model* model = new Model();

    bool success = nvl::modelLoadFromFile(filename, *model);

    if (success) {
        nvl::meshUpdateFaceNormals(model->mesh);
        nvl::meshUpdateVertexNormals(model->mesh);

        ModelDrawer* modelDrawer = new ModelDrawer(model);
        modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
        modelDrawer->meshDrawer().setWireframeVisible(true);

        vModels.push_back(model);
        vModelDrawers.push_back(modelDrawer);

        nvl::FilenameInfo fileInfo = nvl::getFilenameInfo(filename);
        vCanvas->addDrawable(modelDrawer, fileInfo.file);
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error loading model!"));
    }
}

void SkinMixerManager::updateView()
{
    bool drawableSelected = vCanvas->selectedDrawableNumber() > 0;
    ui->modelsRemoveButton->setEnabled(drawableSelected);

    bool singleDrawableSelected = vSelectedModelDrawer != nullptr;
    ui->modelsSaveButton->setEnabled(singleDrawableSelected);

    bool jointSelected = singleDrawableSelected && vSelectedJointId != nvl::MAX_ID;
    bool preview = !vPreviewFaceSegmentation.empty();

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
    vPreviewFaceSegmentation =
            skinmixer::skeletonBinarySegmentationGraphcut(
                *vSelectedModelDrawer->model(),
                compactness,
                vSelectedJointId,
                vPreviewJointSegmentation);

    showModelSegmentationColor();
    updateView();
}

void SkinMixerManager::clearSegmentationPreview()
{
    vPreviewFaceSegmentation.clear();
    vPreviewJointSegmentation.clear();
    resetModelSegmentationColor();
}

void SkinMixerManager::showModelSegmentationColor()
{
    typedef nvl::Model3d Model;

    resetModelSegmentationColor();

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID || vPreviewFaceSegmentation.empty()) {
        return;
    }

    Model* model = vSelectedModelDrawer->model();

    int maxLabel = -1;
    for (const int& l : vPreviewFaceSegmentation) {
        maxLabel = std::max(maxLabel, l);
    }
    for (const int& l : vPreviewJointSegmentation) {
        maxLabel = std::max(maxLabel, l);
    }
    std::vector<nvl::Color> colors = nvl::getDifferentColors(maxLabel + 1);

    for (const auto& face : model->mesh.faces()) {
        vSelectedModelDrawer->meshDrawer().setRenderingFaceColor(face.id(), colors[vPreviewFaceSegmentation[face.id()]]);
    }
    for (const auto& joint : model->skeleton.joints()) {
        vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(joint.id(), colors[vPreviewJointSegmentation[joint.id()]]);
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

void SkinMixerManager::updateSkinningWeightVertexValues()
{
    if (vSelectedModelDrawer != nullptr) {
        std::vector<double> vertexValues;

        if (vSelectedJointId != nvl::MAX_ID) {
            vertexValues.resize(vSelectedModelDrawer->model()->mesh.vertexNumber(), 0.0);

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
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter("Model (*.txt)");

    QStringList files;

    if (dialog.exec())
        files = dialog.selectedFiles();

    for (QString str : files) {
        loadModelFromFile(str.toStdString());
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
    if (!vPreviewFaceSegmentation.empty()) {
        segmentationPreview();
    }
}

void SkinMixerManager::on_detachingDetachButton_clicked()
{
    detachBySegmentation();
}

void SkinMixerManager::on_attachingMoveButton_clicked()
{
    moveModelInPosition();
}

}

