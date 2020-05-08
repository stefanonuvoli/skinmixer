#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/colorize.h>
#include <nvl/utilities/timer.h>

#include <nvl/io/model_io.h>

#include <nvl/models/mesh_normals.h>
#include <nvl/models/model_transformations.h>

#include "skinmixer/detach.h"
#include "skinmixer/attach.h"

#include <QFileDialog>
#include <QMessageBox>
#include <iostream>

namespace skinmixer {

SkinMixerManager::SkinMixerManager(
        nvl::Canvas* canvas,
        nvl::DrawableListWidget* drawableListWidget,
        nvl::SkeletonJointListWidget* skeletonJointListWidget,
        QWidget *parent) :
    QFrame(parent),
    ui(new Ui::SkinMixerManager),
    vCanvas(canvas),
    vDrawableListWidget(drawableListWidget),
    vSkeletonJointListWidget(skeletonJointListWidget),
    vDetachPreview(false),
    vDetachPreviewDrawer(&vDetachPreviewMesh),
    vAttachSelectedModelDrawer1(nullptr),
    vAttachSelectedJoint1(nvl::MAX_ID),
    vAttachSelectedModelDrawer2(nullptr),
    vAttachSelectedJoint2(nvl::MAX_ID)
{
    ui->setupUi(this);

    vDetachPreviewDrawer.setPolylineColorMode(PolylineMeshDrawer::POLYLINE_COLOR_UNIFORM);
    vDetachPreviewDrawer.setPolylineShapeMode(PolylineMeshDrawer::POLYLINE_SHAPE_LINE);
    vDetachPreviewDrawer.setPolylineUniformColor(nvl::Color(200, 50, 50));
    vDetachPreviewDrawer.setPolylineSize(5);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
    vSkinMixerGraph.clear();

    for (ModelDrawer*& drawers : vModelDrawers) {
        if (drawers != nullptr) {
            delete drawers;
            drawers = nullptr;
        }
    }

    delete ui;
}

void SkinMixerManager::slot_picking(const std::vector<long long int>& data) {
    bool shiftPressed = QApplication::keyboardModifiers() & Qt::ShiftModifier;
    if (data.size() >= 1) {
        size_t drawableId = data[0];

        const std::unordered_set<Index>& selectedDrawables = vDrawableListWidget->selectedDrawables();

        if (selectedDrawables.size() != 1 || *selectedDrawables.begin() != drawableId) {
            if (shiftPressed) {
                if (selectedDrawables.find(drawableId) == selectedDrawables.end()) {
                    vDrawableListWidget->selectDrawable(drawableId);
                }
                else {
                    vDrawableListWidget->unselectDrawable(drawableId);
                }
            }
            else {
                vDrawableListWidget->unselectAllDrawables();
                vDrawableListWidget->selectDrawable(drawableId);
            }
        }

        if (data.size() >= 3) {
            long long int pickingIdentifier = data[1];
            if (pickingIdentifier == nvl::Pickable::PICKING_SKELETON_JOINT) {
                Index jointId = data[2];

                vSkeletonJointListWidget->updateJointList();

                const std::unordered_set<Index>& selectedJoints = vSkeletonJointListWidget->selectedJoints();
                if (shiftPressed) {
                    if (selectedJoints.find(jointId) == selectedJoints.end()) {
                        vSkeletonJointListWidget->selectJoint(jointId);
                    }
                    else {
                        vSkeletonJointListWidget->unselectJoint(jointId);
                    }
                }
                else {
                    if (selectedJoints.size() != 1 || *selectedJoints.begin() != jointId) {
                        vSkeletonJointListWidget->unselectAllJoints();
                        vSkeletonJointListWidget->selectJoint(jointId);
                    }
                }
            }
        }
        else {
            vSkeletonJointListWidget->unselectAllJoints();
        }
    }
}

void SkinMixerManager::slot_jointSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedJoints)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedJoints);

    vSelectedJointId = nvl::MAX_ID;

    if (vSelectedModelDrawer != nullptr) {
        const std::unordered_set<nvl::Skeleton3d::JointId>& selectionData = vSkeletonJointListWidget->selectedJoints();
        if (selectionData.size() == 1) {
            vSelectedJointId = static_cast<nvl::Skeleton3d::JointId>(*selectionData.begin());
        }

        updateSkinningWeightVertexValues();
    }

    if (vDetachPreview) {
        updateDetachPreview();
    }

    updateView();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    clearDetachPreview();

    vSelectedModelDrawer = nullptr;
    vSelectedJointId = nvl::MAX_ID;

    const std::unordered_set<Index>& selectedItems = vDrawableListWidget->selectedDrawables();
    if (selectedItems.size() == 1) {
        Index firstItem = *selectedItems.begin();
        nvl::Drawable* drawable = vCanvas->drawable(firstItem);

        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(drawable);
        if (modelDrawer != nullptr) {
            vSelectedModelDrawer = modelDrawer;
        }
    }

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    updateSkinningWeightVertexValues();

    updateView();
}

void SkinMixerManager::slot_movableFrameChanged()
{
    nvl::Quaterniond rot(vCanvas->movableFrame().rotation());
    nvl::Translation3d tra(vCanvas->movableFrame().translation());

    for (Index id : vDrawableListWidget->selectedDrawables()) {
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

    std::vector<nvl::Segment<Point>> functionSegments;

    nvl::Timer t("Detaching timer");
    std::vector<Index> newNodes =
            skinmixer::detach(
                vSkinMixerGraph,
                vSelectedModelDrawer->model(),
                vSelectedJointId,
                ui->detachingOffsetSpinBox->value(),
                ui->detachingRigiditySpinBox->value(),
                ui->detachingKeepSkeletonCheckBox->isChecked(),
                ui->detachingSmoothCheckBox->isChecked());
    t.print();

    if (!newNodes.empty()) {
        for (Index newNode : newNodes) {
            Index id = addModelDrawerFromNode(newNode, "Detaching " + std::to_string(newNode));
            vModelDrawers[id]->setFrame(vSelectedModelDrawer->frame());
        }

        vCanvas->removeDrawable(vSelectedModelDrawer);
        assert(vCanvas->drawableNumber() > 0);

        vDrawableListWidget->selectDrawable(vCanvas->drawableNumber() - 1);

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

    if (vDetachPreview) {
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
                ui->detachingRigiditySpinBox->value(),
                ui->detachingSmoothCheckBox->isChecked());

    vDetachPreviewMesh.clear();

    std::map<Point, PolylineVertexId> polylineVerticesMap;

    for (const nvl::Segment<Point>& segment : functionSegments) {
        if (polylineVerticesMap.find(segment.p1()) == polylineVerticesMap.end()) {
            polylineVerticesMap.insert(std::make_pair(segment.p1(), vDetachPreviewMesh.addVertex(segment.p1())));
        }

        if (polylineVerticesMap.find(segment.p2()) == polylineVerticesMap.end()) {
            polylineVerticesMap.insert(std::make_pair(segment.p2(), vDetachPreviewMesh.addVertex(segment.p2())));
        }
    }

    for (const nvl::Segment<Point>& segment : functionSegments) {
        assert(polylineVerticesMap.find(segment.p1()) != polylineVerticesMap.end());
        assert(polylineVerticesMap.find(segment.p2()) != polylineVerticesMap.end());
        vDetachPreviewMesh.addPolyline(polylineVerticesMap.at(segment.p1()), polylineVerticesMap.at(segment.p2()));
    }

    vDetachPreviewDrawer.setFrame(vSelectedModelDrawer->frame());

    vDetachPreviewDrawer.update();

    vCanvas->addDrawable(&vDetachPreviewDrawer, "Detach preview");
    vCanvas->updateGL();

    vDetachPreview = true;

    updateView();
}

void SkinMixerManager::clearDetachPreview()
{
    if (vDetachPreview) {
        vDetachPreviewMesh.clear();

        vCanvas->removeDrawable(&vDetachPreviewDrawer);
        vCanvas->updateGL();

        vDetachPreview = false;

        updateView();
    }
}

void SkinMixerManager::clearAttachSelection()
{
    vAttachSelectedModelDrawer1 = nullptr;
    vAttachSelectedJoint1 = nvl::MAX_ID;
    vAttachSelectedModelDrawer2 = nullptr;
    vAttachSelectedJoint2 = nvl::MAX_ID;

    updateView();
}

void SkinMixerManager::updateView()
{
    bool drawableSelected = vDrawableListWidget->selectedDrawableNumber() > 0;
    ui->modelsRemoveButton->setEnabled(drawableSelected);

    bool singleDrawableSelected = vSelectedModelDrawer != nullptr;
    ui->modelsSaveButton->setEnabled(singleDrawableSelected);

    bool jointSelected = singleDrawableSelected && vSelectedJointId != nvl::MAX_ID;

    bool attachingSelected1 = vAttachSelectedJoint1 != nvl::MAX_ID;
    bool attachingSelected2 = vAttachSelectedJoint2 != nvl::MAX_ID;

    ui->detachingPreviewButton->setEnabled(jointSelected && !vDetachPreview);
    ui->detachingClearButton->setEnabled(vDetachPreview);
    ui->detachingDetachButton->setEnabled(jointSelected);

    ui->attachingSelect1Button->setEnabled(!attachingSelected1);
    ui->attachingSelect2Button->setEnabled(attachingSelected1 && !attachingSelected2);
    ui->attachingAbortButton->setEnabled(attachingSelected1 || attachingSelected2);
    ui->attachingFindPositionButton->setEnabled(attachingSelected2);
    ui->attachingAttachButton->setEnabled(attachingSelected2);

    ui->transformationMoveButton->setEnabled(singleDrawableSelected);
}

void SkinMixerManager::attachFindPosition()
{
    if (vAttachSelectedJoint1 == nvl::MAX_ID || vAttachSelectedJoint2 == nvl::MAX_ID) {
        return;
    }

    Model* model1 = vAttachSelectedModelDrawer1->model();
    Model* model2 = vAttachSelectedModelDrawer2->model();

    nvl::Index nodeId1 = vSkinMixerGraph.nodeId(model1);
    nvl::Index nodeId2 = vSkinMixerGraph.nodeId(model2);

    nvl::Affine3d frameTransformation1 = vAttachSelectedModelDrawer1->frame();
    nvl::Affine3d frameTransformation2 = vAttachSelectedModelDrawer2->frame();

    nvl::Affine3d transformation2 = skinmixer::attachFindTransformation(
                vSkinMixerGraph,
                nodeId1,
                nodeId2,
                vAttachSelectedJoint1,
                vAttachSelectedJoint2,
                frameTransformation1,
                frameTransformation2);

    vAttachSelectedModelDrawer2->setFrame(transformation2 * vAttachSelectedModelDrawer2->frame());
    vAttachSelectedModelDrawer2->update();

    vCanvas->updateGL();
}

void SkinMixerManager::doAttach()
{

    Model* model1 = vAttachSelectedModelDrawer1->model();
    Model* model2 = vAttachSelectedModelDrawer2->model();

    nvl::Index nodeId1 = vSkinMixerGraph.nodeId(model1);
    nvl::Index nodeId2 = vSkinMixerGraph.nodeId(model2);

    nvl::Affine3d frameTransformation1 = vAttachSelectedModelDrawer1->frame();
    nvl::Affine3d frameTransformation2 = vAttachSelectedModelDrawer2->frame();

    nvl::Timer t("Attaching timer");
    std::vector<Index> newNodes =
            skinmixer::attach(
                vSkinMixerGraph,
                nodeId1,
                nodeId2,
                vAttachSelectedJoint1,
                vAttachSelectedJoint2,
                frameTransformation1,
                frameTransformation2);
    t.print();

    if (!newNodes.empty()) {
        for (Index newNode : newNodes) {
            Index id = addModelDrawerFromNode(newNode, "Attaching " + std::to_string(newNode));
            vModelDrawers[id]->setFrame(vSelectedModelDrawer->frame());
        }

        vCanvas->removeDrawable(vAttachSelectedModelDrawer1);
        vCanvas->removeDrawable(vAttachSelectedModelDrawer2);
        assert(vCanvas->drawableNumber() > 0);

        vDrawableListWidget->selectDrawable(vCanvas->drawableNumber() - 1);

        clearDetachPreview();

        vCanvas->updateGL();
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error detaching the model! Probably you have selected a root or a final joint."));
    }
}

void SkinMixerManager::nodeApplyFrameTransformation()
{
    if (vSelectedModelDrawer == nullptr) {
        return;
    }

    Model* model = vSelectedModelDrawer->model();

    nvl::Affine3d frameTransformation = vSelectedModelDrawer->frame();

    //Apply transformation to the node
    skinmixer::applyTransformationToNode(vSkinMixerGraph.node(model), frameTransformation);

    vSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());
    vSelectedModelDrawer->update();

    vCanvas->updateGL();
}

void SkinMixerManager::loadModelFromFile(const std::string& filename)
{
    Model model;

    bool success = nvl::modelLoadFromFile(filename, model);

    if (success) {
        Index nodeId = vSkinMixerGraph.addNode(model, OperationType::NONE);
        addModelDrawerFromNode(nodeId, filename);
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error loading model!"));
    }
}

SkinMixerManager::Index SkinMixerManager::addModelDrawerFromNode(const Index& nodeId, const std::string& name)
{
    return addModelDrawerFromNode(vSkinMixerGraph.node(nodeId), name);
}

SkinMixerManager::Index SkinMixerManager::addModelDrawerFromNode(SkinMixerNode& node, const std::string& name)
{
    Model* model = node.model;

    nvl::meshUpdateFaceNormals(model->mesh);
    nvl::meshUpdateVertexNormals(model->mesh);
    vModels.push_back(model);

    ModelDrawer* modelDrawer = new ModelDrawer(model);
    modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
    modelDrawer->meshDrawer().setWireframeVisible(true);
    modelDrawer->meshDrawer().setPickable(false);
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
        connect(vDrawableListWidget, &nvl::DrawableListWidget::signal_drawableSelectionChanged, this, &SkinMixerManager::slot_drawableSelectionChanged);
        connect(vSkeletonJointListWidget, &nvl::SkeletonJointListWidget::signal_jointSelectionChanged, this, &SkinMixerManager::slot_jointSelectionChanged);
        connect(vCanvas, &nvl::Canvas::signal_movableFrameChanged, this, &SkinMixerManager::slot_movableFrameChanged);
        connect(vCanvas, &nvl::Canvas::signal_picking, this, &SkinMixerManager::slot_picking);
    }
}

void SkinMixerManager::on_modelsLoadButton_clicked()
{
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter("Model (*.mdl)");

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
    while (!vDrawableListWidget->selectedDrawables().empty()) {
        vCanvas->removeDrawable(*vDrawableListWidget->selectedDrawables().begin());
    }
    vCanvas->updateGL();
}

void SkinMixerManager::on_detachingPreviewButton_clicked()
{
    updateDetachPreview();

    clearAttachSelection();
}

void SkinMixerManager::on_detachingClearButton_clicked()
{
    clearDetachPreview();

    clearAttachSelection();
}

void SkinMixerManager::on_detachingOffsetSpinBox_valueChanged(double arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (vDetachPreview) {
        clearDetachPreview();
        updateDetachPreview();
    }
}

void SkinMixerManager::on_detachingRigiditySpinBox_valueChanged(double arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (vDetachPreview) {
        clearDetachPreview();
        updateDetachPreview();
    }
}

void SkinMixerManager::on_detachingSmoothCheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    if (vDetachPreview) {
        clearDetachPreview();
        updateDetachPreview();
    }
}

void SkinMixerManager::on_detachingDetachButton_clicked()
{
    clearAttachSelection();

    doDetach();
}

void SkinMixerManager::on_transformationMoveButton_clicked()
{
    clearAttachSelection();

    nodeApplyFrameTransformation();
}

void SkinMixerManager::on_attachingSelect1Button_clicked()
{
    assert(vAttachSelectedJoint1 == nvl::MAX_ID && vAttachSelectedJoint2 == nvl::MAX_ID);

    clearDetachPreview();

    if (vSelectedModelDrawer != nullptr && vSelectedJointId != nvl::MAX_ID) {
        vAttachSelectedModelDrawer1 = vSelectedModelDrawer;
        vAttachSelectedJoint1 = vSelectedJointId;
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Select the joint of a model."));
    }

    updateView();
}

void SkinMixerManager::on_attachingSelect2Button_clicked()
{
    assert(vAttachSelectedJoint1 != nvl::MAX_ID && vAttachSelectedJoint2 == nvl::MAX_ID);

    clearDetachPreview();

    if (vSelectedModelDrawer != nullptr && vSelectedJointId != nvl::MAX_ID) {
        if (vAttachSelectedModelDrawer1 != vSelectedModelDrawer) {
            vAttachSelectedModelDrawer2 = vSelectedModelDrawer;
            vAttachSelectedJoint2 = vSelectedJointId;
        }
        else {
            QMessageBox::warning(this, tr("SkinMixer"), tr("You cannot select the same model!"));
        }
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Select the joint of a model."));
    }
    updateView();
}

void SkinMixerManager::on_attachingAbortButton_clicked()
{
    assert(vAttachSelectedJoint1 != nvl::MAX_ID || vAttachSelectedJoint2 != nvl::MAX_ID);

    clearDetachPreview();

    clearAttachSelection();
}

void SkinMixerManager::on_attachingFindPositionButton_clicked()
{
    assert(vAttachSelectedJoint1 != nvl::MAX_ID && vAttachSelectedJoint2 != nvl::MAX_ID);

    clearDetachPreview();

    attachFindPosition();

    updateView();
}

void SkinMixerManager::on_attachingAttachButton_clicked()
{
    assert(vAttachSelectedJoint1 != nvl::MAX_ID && vAttachSelectedJoint2 != nvl::MAX_ID);

    clearDetachPreview();

    //TODO

    clearAttachSelection();
}

}
