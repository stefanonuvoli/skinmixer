#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/colorize.h>
#include <nvl/utilities/timer.h>

#include <nvl/io/model_io.h>

#include <nvl/models/mesh_normals.h>
#include <nvl/models/model_transformations.h>

#include "skinmixer/detach.h"

#include <QFileDialog>
#include <QMessageBox>
#include <iostream>

namespace skinmixer {

SkinMixerManager::SkinMixerManager(nvl::Canvas* canvas, QWidget *parent) :
    QFrame(parent),
    ui(new Ui::SkinMixerManager),
    vCanvas(canvas),
    vSelectedModelDrawer(nullptr),
    vSelectedJointId(nvl::MAX_ID),
    detachPreview(false),
    detachPreviewDrawer(&detachPreviewMesh)
{
    ui->setupUi(this);

    detachPreviewDrawer.setPolylineColorMode(PolylineMeshDrawer::POLYLINE_COLOR_UNIFORM);
    detachPreviewDrawer.setPolylineShapeMode(PolylineMeshDrawer::POLYLINE_SHAPE_LINE);
    detachPreviewDrawer.setPolylineUniformColor(nvl::Color(200, 50, 50));
    detachPreviewDrawer.setPolylineSize(10);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
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
            vSelectedJointId = static_cast<Index>(*selectionData.begin());
        }

        updateSkinningWeightVertexValues();
    }

    if (detachPreview) {
        updateDetachPreview();
    }

    updateView();
}

void SkinMixerManager::slot_drawableSelectionChanged()
{
    clearDetachPreview();

    vSelectedModelDrawer = nullptr;
    vSelectedJointId = nvl::MAX_ID;

    const std::unordered_set<Index>& selectedItems = vCanvas->selectedDrawables();
    if (selectedItems.size() == 1) {
        Index firstItem = *selectedItems.begin();
        nvl::Drawable* drawable = vCanvas->drawable(firstItem);

        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(drawable);
        if (modelDrawer != nullptr) {
            vSelectedModelDrawer = modelDrawer;

            const std::vector<long long int>& selectionData = vSelectedModelDrawer->skeletonDrawer().selectionData();
            if (selectionData.size() == 1) {
                vSelectedJointId = static_cast<Index>(*selectionData.begin());
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

    for (Index id : vCanvas->selectedDrawables()) {
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

void SkinMixerManager::doDetach()
{
    typedef Model::Mesh::Point Point;

    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    std::vector<std::vector<Model::Mesh::VertexId>> vertexMaps;
    std::vector<std::vector<Model::Mesh::FaceId>> faceMaps;
    std::vector<std::vector<Model::Skeleton::JointId>> jointMaps;

    std::vector<nvl::Segment<Point>> functionSegments;

    nvl::Timer t("Detaching timer");
    std::vector<Index> newNodes =
            skinmixer::detach(
                skinMixerGraph,
                vSelectedModelDrawer->model(),
                vSelectedJointId,
                ui->detachingOffsetSpinBox->value(),
                ui->detachingKeepSkeletonCheckBox->isChecked(),
                ui->detachingSmoothCheckBox->isChecked());
    t.print();

    if (!newNodes.empty()) {
        for (Index newNode : newNodes) {
            Index id = addModelFromNode(newNode, "Detaching " + std::to_string(newNode));
            vModelDrawers[id]->setFrame(vSelectedModelDrawer->frame());
        }

        vCanvas->removeDrawable(vSelectedModelDrawer);
        vCanvas->select(vCanvas->drawableNumber() - 1);

        clearDetachPreview();

        vCanvas->updateGL();
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error detaching the model! Probably you have selected a root or a final joint."));
    }
}

void SkinMixerManager::updateDetachPreview()
{
    typedef Model::Mesh::Point Point;
    typedef PolylineMesh::VertexId PolylineVertexId;

    if (detachPreview) {
        clearDetachPreview();
    }


    if (vSelectedModelDrawer == nullptr || vSelectedJointId == nvl::MAX_ID) {
        return;
    }

    std::vector<std::vector<Model::Mesh::VertexId>> birthVertex;
    std::vector<std::vector<Model::Mesh::FaceId>> birthFace;
    std::vector<std::vector<Model::Skeleton::JointId>> birthJoint;

    std::vector<nvl::Segment<Point>> functionSegments =
            skinmixer::detachPreview(
                *vSelectedModelDrawer->model(),
                vSelectedJointId,
                ui->detachingOffsetSpinBox->value(),
                ui->detachingSmoothCheckBox->isChecked());

    detachPreviewMesh.clear();

    std::map<Point, PolylineVertexId> polylineVerticesMap;

    for (const nvl::Segment<Point>& segment : functionSegments) {
        if (polylineVerticesMap.find(segment.p1()) == polylineVerticesMap.end()) {
            polylineVerticesMap.insert(std::make_pair(segment.p1(), detachPreviewMesh.addVertex(segment.p1())));
        }

        if (polylineVerticesMap.find(segment.p2()) == polylineVerticesMap.end()) {
            polylineVerticesMap.insert(std::make_pair(segment.p2(), detachPreviewMesh.addVertex(segment.p2())));
        }
    }

    for (const nvl::Segment<Point>& segment : functionSegments) {
        assert(polylineVerticesMap.find(segment.p1()) != polylineVerticesMap.end());
        assert(polylineVerticesMap.find(segment.p2()) != polylineVerticesMap.end());
        detachPreviewMesh.addPolyline(polylineVerticesMap.at(segment.p1()), polylineVerticesMap.at(segment.p2()));
    }

    detachPreviewDrawer.setFrame(vSelectedModelDrawer->frame());

    detachPreviewDrawer.update();

    vCanvas->addDrawable(&detachPreviewDrawer, "Detach preview");
    vCanvas->updateGL();

    detachPreview = true;

    updateView();
}

void SkinMixerManager::clearDetachPreview()
{
    if (detachPreview) {
        detachPreviewMesh.clear();

        vCanvas->removeDrawable(&detachPreviewDrawer);
        vCanvas->updateGL();

        detachPreview = false;

        updateView();
    }
}

void SkinMixerManager::updateView()
{
    bool drawableSelected = vCanvas->selectedDrawableNumber() > 0;
    ui->modelsRemoveButton->setEnabled(drawableSelected);

    bool singleDrawableSelected = vSelectedModelDrawer != nullptr;
    ui->modelsSaveButton->setEnabled(singleDrawableSelected);

    bool jointSelected = singleDrawableSelected && vSelectedJointId != nvl::MAX_ID;

    ui->detachingPreviewButton->setEnabled(jointSelected && !detachPreview);
    ui->detachingClearButton->setEnabled(detachPreview);
    ui->detachingDetachButton->setEnabled(jointSelected);
}

void SkinMixerManager::moveModelInPosition()
{
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

SkinMixerManager::Model* SkinMixerManager::loadModelFromFile(const std::string& filename)
{
    if (filename.empty())
        return nullptr;

    Model* model = new Model();

    bool success = nvl::modelLoadFromFile(filename, *model);

    if (success) {
        Index nodeId = skinMixerGraph.addNode(model, OperationType::NONE);

        addModelFromNode(nodeId, filename);

        return model;
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error loading model!"));
        return nullptr;
    }
}

SkinMixerManager::Index SkinMixerManager::addModelFromNode(const Index& nodeId, const std::string& name)
{
    return addModelFromNode(skinMixerGraph.node(nodeId), name);
}

SkinMixerManager::Index SkinMixerManager::addModelFromNode(SkinMixerNode& node, const std::string& name)
{
    Model* model = node.model;

    nvl::meshUpdateFaceNormals(model->mesh);
    nvl::meshUpdateVertexNormals(model->mesh);
    vModels.push_back(model);

    ModelDrawer* modelDrawer = new ModelDrawer(model);
    modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
    modelDrawer->meshDrawer().setWireframeVisible(true);
    vModelDrawers.push_back(modelDrawer);

    nvl::FilenameInfo fileInfo = nvl::getFilenameInfo(name);
    vCanvas->addDrawable(modelDrawer, fileInfo.file);

    return vModels.size() - 1;
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
    while (!vCanvas->selectedDrawables().empty()) {
        vCanvas->removeDrawable(*vCanvas->selectedDrawables().begin());
    }
    vCanvas->updateGL();
}

void SkinMixerManager::on_detachingPreviewButton_clicked()
{
    updateDetachPreview();
}

void SkinMixerManager::on_detachingClearButton_clicked()
{
    clearDetachPreview();
}

void SkinMixerManager::on_detachingOffsetSpinBox_valueChanged(double arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (detachPreview) {
        clearDetachPreview();
        updateDetachPreview();
    }
}

void SkinMixerManager::on_detachingSmoothCheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (detachPreview) {
        clearDetachPreview();
        updateDetachPreview();
    }
}

void SkinMixerManager::on_detachingDetachButton_clicked()
{
    doDetach();
}

void SkinMixerManager::on_attachingMoveButton_clicked()
{
    moveModelInPosition();
}


}
