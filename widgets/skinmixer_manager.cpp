#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/timer.h>

#include <nvl/utilities/colorize.h>

#include <QFileDialog>
#include <QMessageBox>
#include <iostream>

#include "skinmixer/skinmixer_cut.h"

SkinMixerManager::SkinMixerManager(
        nvl::Canvas* canvas,
        nvl::DrawableListWidget* drawableListWidget,
        nvl::SkeletonJointListWidget* skeletonJointListWidget,
        nvl::ModelLoaderWidget* modelLoaderWidget,
        QWidget *parent) :
    QFrame(parent),
    ui(new Ui::SkinMixerManager),
    vCanvas(canvas),
    vDrawableListWidget(drawableListWidget),
    vSkeletonJointListWidget(skeletonJointListWidget),
    vModelLoaderWidget(modelLoaderWidget),
    vCutPreviewDrawer(&vCutPreviewMesh),
    lastSelectedModelDrawer(nullptr)
{
    ui->setupUi(this);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
    delete ui;
}

void SkinMixerManager::slot_canvasPicking(const std::vector<PickingData>& data) {
    bool shiftPressed = QApplication::keyboardModifiers() & Qt::ShiftModifier;

    if (data.size() > 0) {
        Index selected = 0;
        for (Index i = 0; i < data.size(); ++i) {
            if (data[i].identifier == nvl::Canvas::PICKING_SKELETON_JOINT) {
                selected = i;
                break;
            }
        }

        const PickingData& picked = data[selected];
        size_t drawableId = picked.value1;

        std::unordered_set<Index> selectedDrawables = vDrawableListWidget->selectedDrawables();

        if (selectedDrawables.size() != 1 || *selectedDrawables.begin() != drawableId) {
            if (shiftPressed) {
                if (selectedDrawables.find(drawableId) == selectedDrawables.end()) {
                    selectedDrawables.insert(drawableId);
                }
                else {
                    selectedDrawables.erase(drawableId);
                }
            }
            else {
                selectedDrawables.clear();
                selectedDrawables.insert(drawableId);
            }

            vDrawableListWidget->setSelectedDrawables(selectedDrawables);
        }

        std::unordered_set<Index> selectedJoints;

        if (picked.identifier == nvl::Canvas::PICKING_SKELETON_JOINT) {
            Index jointId = picked.value2;

            vSkeletonJointListWidget->updateJointList();

            selectedJoints = vSkeletonJointListWidget->selectedJoints();

            if (shiftPressed) {
                if (selectedJoints.find(jointId) == selectedJoints.end()) {
                    selectedJoints.insert(jointId);
                }
                else {
                    selectedJoints.erase(jointId);
                }
            }
            else {
                if (selectedJoints.size() != 1 || *selectedJoints.begin() != jointId) {
                    selectedJoints.clear();
                    selectedJoints.insert(jointId);
                }
            }
        }
        else {
            selectedJoints.clear();
        }

        vSkeletonJointListWidget->setSelectedJoints(selectedJoints);
    }
}

void SkinMixerManager::slot_jointSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedJoints)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedJoints);

    updateView();

    updateSelectionColorization();
    updatePreview();

    updateSkinningWeightVertexValues();

    vCanvas->updateGL();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    updateView();

    updateSelectionColorization();
    updatePreview();

    updateSkinningWeightVertexValues();

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    vCanvas->updateGL();
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

void SkinMixerManager::slot_drawableAdded(const SkinMixerManager::Index& id, nvl::Drawable* drawable)
{
    if (vCanvas->isPickable(id)) {
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(drawable);
        if (modelDrawer != nullptr) {
            modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);

            modelDrawer->skeletonDrawer().setVisible(true);
            modelDrawer->skeletonDrawer().setJointSize(8);

            //Copy model
            Model* model = new Model(*(modelDrawer->model()));
            vModels.push_back(model);

            //Add the model to the graph and map the model to the node id
            Index nodeId = vOperationGraph.addNode(model);
            vNodeMap.insert(std::make_pair(modelDrawer->model(), nodeId));
        }
    }
}

void SkinMixerManager::doRemove()
{
    doCutOperation(Operation::REMOVE);
}

void SkinMixerManager::doDetach()
{
    doCutOperation(Operation::DETACH);
}

void SkinMixerManager::doSplit()
{
    doCutOperation(Operation::SPLIT);
}

void SkinMixerManager::doCutOperation(const Operation& operation)
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    assert(selectedModelDrawer != nullptr && selectedJointId != nvl::MAX_ID);

    const double offset = ui->cutOffsetSlider->value() /100.0;
    const double rigidity = ui->cutRigiditySlider->value() / 100.0;
    const bool smooth = ui->cutSmoothCheckBox->isChecked();

    Model* modelPtr = selectedModelDrawer->model();
    Index nodeId = vNodeMap.at(modelPtr);

    nvl::Timer t("Cut operation timer");
    std::vector<Index> newNodes =
            skinmixer::cutOperation(
                vOperationGraph,
                nodeId,
                operation,
                selectedJointId,
                offset,
                rigidity,
                smooth,
                false);
    t.print();

    if (!newNodes.empty()) {
        for (Index newNode : newNodes) {
            vModelLoaderWidget->loadModel(vOperationGraph.node(newNode).model, "Cut " + std::to_string(newNode));
        }

        selectedModelDrawer->setVisible(false);

        vDrawableListWidget->unselectAllDrawables();
        vDrawableListWidget->selectDrawable(vCanvas->drawableNumber() - 1);

        updatePreview();

        vCanvas->updateGL();
    }
    else {
        QMessageBox::warning(this, tr("SkinMixer"), tr("Error detaching the model! Probably you have selected a root or a final joint."));
    }
}

SkinMixerManager::ModelDrawer *SkinMixerManager::getSelectedModelDrawer()
{
    ModelDrawer* selectedModelDrawer = nullptr;

    const std::unordered_set<Index>& selectedDrawables = vDrawableListWidget->selectedDrawables();
    if (selectedDrawables.size() == 1) {
        Index firstItem = *selectedDrawables.begin();
        nvl::Drawable* drawable = vCanvas->drawable(firstItem);

        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(drawable);
        if (modelDrawer != nullptr) {
            selectedModelDrawer = modelDrawer;
        }
    }

    return selectedModelDrawer;
}

SkinMixerManager::JointId SkinMixerManager::getSelectedJointId()
{
    SkinMixerManager::JointId selectedJointId = nvl::MAX_ID;

    const std::unordered_set<Index>& selectedJoints = vSkeletonJointListWidget->selectedJoints();
    if (selectedJoints.size() == 1) {
        Index firstItem = *selectedJoints.begin();
        selectedJointId = firstItem;
    }

    return selectedJointId;
}

void SkinMixerManager::updateView()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    bool drawableSelected = selectedModelDrawer != nullptr;
    bool jointSelected = drawableSelected && selectedJointId != nvl::MAX_ID;

    bool cutOperation = ui->operationDetachRadio->isChecked() || ui->operationRemoveRadio->isChecked() || ui->operationSplitRadio->isChecked();

    ui->operationApplyButton->setEnabled(jointSelected);
    ui->cutPage->setEnabled(cutOperation);
}

void SkinMixerManager::updatePreview()
{
    typedef Model::Mesh Mesh;
    typedef Mesh::Point Point;
    typedef Mesh::FaceId FaceId;
    typedef PolylineMesh::VertexId PolylineVertexId;

    this->vCutPreviewMesh.clear();
    this->vCutPreviewDrawer.resetFrame();

    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    bool drawableSelected = selectedModelDrawer != nullptr;
    bool jointSelected = drawableSelected && selectedJointId != nvl::MAX_ID;

    if (ui->operationPreviewCheckBox->isChecked() && jointSelected) {
        bool cutOperation = ui->operationDetachRadio->isChecked() || ui->operationRemoveRadio->isChecked() || ui->operationSplitRadio->isChecked();

        if (cutOperation) {
            selectedModelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
            Model& model = *selectedModelDrawer->model();
            Mesh& mesh = model.mesh;

            const double offset = ui->cutOffsetSlider->value() /100.0;
            const double rigidity = ui->cutRigiditySlider->value() / 100.0;
            const bool smooth = ui->cutSmoothCheckBox->isChecked();

            Operation operation = Operation::NONE;
            if (ui->operationDetachRadio->isChecked()) {
                operation = Operation::DETACH;
            }
            if (ui->operationRemoveRadio->isChecked()) {
                operation = Operation::REMOVE;
            }
            if (ui->operationSplitRadio->isChecked()) {
                operation = Operation::SPLIT;
            }
            assert(operation != Operation::NONE);

            std::vector<typename Model::SkinningWeights::Scalar> cutFunction;
            std::vector<std::vector<nvl::Segment<Point>>> cutSegments =
                    skinmixer::cutOperationPreview(
                        model,
                        selectedJointId,
                        offset,
                        rigidity,
                        smooth,
                        cutFunction);

            for (FaceId vId = 0; vId < mesh.nextVertexId(); vId++) {
                if (mesh.isFaceDeleted(vId))
                    continue;

                double normalizedValue = ((cutFunction[vId] + 1.0) / 2.0);


                if (operation == Operation::REMOVE) {
                    normalizedValue = 1 - normalizedValue;
                }

                nvl::Color c = nvl::getRampRedGreen(normalizedValue, 0.5, 0.8);

                if (normalizedValue >= 0.5) {
                    c = nvl::Color(0.5, 0.8, 0.5);
                }
                else {
                    c = nvl::Color(0.8, 0.5, 0.5);
                }

                selectedModelDrawer->meshDrawer().setRenderingVertexColor(vId, c);
            }

            this->vCutPreviewMesh.clear();

            std::map<Point, PolylineVertexId> polylineVerticesMap;

            for (const std::vector<nvl::Segment<Point>>& componentSegment : cutSegments) {
                for (const nvl::Segment<Point>& segment : componentSegment) {
                    if (polylineVerticesMap.find(segment.p1()) == polylineVerticesMap.end()) {
                        polylineVerticesMap.insert(std::make_pair(segment.p1(), this->vCutPreviewMesh.addVertex(segment.p1())));
                    }

                    if (polylineVerticesMap.find(segment.p2()) == polylineVerticesMap.end()) {
                        polylineVerticesMap.insert(std::make_pair(segment.p2(), this->vCutPreviewMesh.addVertex(segment.p2())));
                    }
                }

                for (const nvl::Segment<Point>& segment : componentSegment) {
                    assert(polylineVerticesMap.find(segment.p1()) != polylineVerticesMap.end());
                    assert(polylineVerticesMap.find(segment.p2()) != polylineVerticesMap.end());
                    vCutPreviewMesh.addPolyline(polylineVerticesMap.at(segment.p1()), polylineVerticesMap.at(segment.p2()));
                }
            }

            this->vCutPreviewDrawer.setFrame(selectedModelDrawer->frame());
        }
    }

    this->vCutPreviewDrawer.update();
}

void SkinMixerManager::updateSkinningWeightVertexValues()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    if (selectedModelDrawer != nullptr) {
        std::vector<double> vertexValues;

        if (selectedJointId != nvl::MAX_ID) {
            vertexValues.resize(selectedModelDrawer->model()->mesh.vertexNumber(), 0.0);

            for (auto vertex : selectedModelDrawer->model()->mesh.vertices()) {
                vertexValues[vertex.id()] = selectedModelDrawer->model()->skinningWeights.weight(vertex.id(), selectedJointId);
            }
        }

        selectedModelDrawer->meshDrawer().setVertexValues(vertexValues);
    }
}

void SkinMixerManager::updateSelectionColorization()
{
    if (this->lastSelectedModelDrawer != nullptr) {
        this->lastSelectedModelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
        this->lastSelectedModelDrawer->meshDrawer().resetRenderingFaceColors();
        this->lastSelectedModelDrawer->skeletonDrawer().resetRenderingJointColors();
    }

    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    if (selectedModelDrawer != nullptr && selectedJointId != nvl::MAX_ID) {
        selectedModelDrawer->skeletonDrawer().setRenderingJointColor(selectedJointId, nvl::Color(1.0, 1.0, 0.0));
    }

    this->lastSelectedModelDrawer = selectedModelDrawer;
}

void SkinMixerManager::initialize()
{
    this->vCutPreviewDrawer.setPolylineColorMode(PolylineMeshDrawer::POLYLINE_COLOR_UNIFORM);
    this->vCutPreviewDrawer.setPolylineShapeMode(PolylineMeshDrawer::POLYLINE_SHAPE_LINE);
    this->vCutPreviewDrawer.setPolylineUniformColor(nvl::Color(0.8, 0.8, 0.3));
    this->vCutPreviewDrawer.setPolylineSize(5);

    vCanvas->addDrawable(&this->vCutPreviewDrawer, "", false);

    updateView();
}

void SkinMixerManager::connectSignals()
{
    if (vCanvas != nullptr) {
        //Connect signals to the viewer
        connect(vDrawableListWidget, &nvl::DrawableListWidget::signal_drawableSelectionChanged, this, &SkinMixerManager::slot_drawableSelectionChanged);
        connect(vSkeletonJointListWidget, &nvl::SkeletonJointListWidget::signal_jointSelectionChanged, this, &SkinMixerManager::slot_jointSelectionChanged);
        connect(vCanvas, &nvl::Canvas::signal_movableFrameChanged, this, &SkinMixerManager::slot_movableFrameChanged);
        connect(vCanvas, &nvl::Canvas::signal_canvasPicking, this, &SkinMixerManager::slot_canvasPicking);
        connect(vCanvas, &nvl::Canvas::signal_drawableAdded, this, &SkinMixerManager::slot_drawableAdded);
    }
}

void SkinMixerManager::on_operationRemoveRadio_toggled(bool checked)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(checked);
    updateView();
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationDetachRadio_toggled(bool checked)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(checked);
    updateView();
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationSplitRadio_toggled(bool checked)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(checked);
    updateView();
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAddRadio_toggled(bool checked)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(checked);
    updateView();
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationReplaceRadio_toggled(bool checked)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(checked);
    updateView();
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationApplyButton_clicked()
{
    if (ui->operationRemoveRadio->isChecked()) {
        doRemove();
    }
    else if (ui->operationDetachRadio->isChecked()) {
        doDetach();
    }
    else if (ui->operationSplitRadio->isChecked()) {
        doSplit();
    }
    else if (ui->operationAddRadio->isChecked()) {
//        doAdd();
    }
    else if (ui->operationReplaceRadio->isChecked()) {
//        doReplace();
    }
}

void SkinMixerManager::on_operationPreviewCheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetSlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutRigiditySlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutSmoothCheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetRigidityButton_clicked()
{
    ui->cutRigiditySlider->setValue(60);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetResetButton_clicked()
{
    ui->cutOffsetSlider->setValue(0);
    updateSelectionColorization();
    updatePreview();

    vCanvas->updateGL();

}
