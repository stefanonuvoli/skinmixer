#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/timer.h>
#include <nvl/utilities/colorize.h>

#include <nvl/io/model_io.h>

#include <QFileDialog>
#include <QMessageBox>

#include <iostream>

#include "skinmixer/skinmixer.h"

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
    currentOperation(OperationType::NONE),
    lastSelectedModelDrawer(nullptr),
    attachFirstModelDrawer(nullptr),
    attachFirstJointId(nvl::MAX_ID),
    attachBackupFrame(nvl::Affine3d::Identity())
{
    ui->setupUi(this);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
    for (ModelDrawer* modelDrawer : vModelDrawers) {
        delete modelDrawer;
    }
    vModelDrawers.clear();

    for (Model* model : vModels) {
        delete model;
    }
    vModels.clear();

    delete ui;
}

nvl::Index SkinMixerManager::loadModelFromFile(const std::string& filename)
{
    Model tmpModel;

    bool success = nvl::modelLoadFromFile(filename, tmpModel);

    if (success) {
        Model* model = new Model(tmpModel);
        updateModelNormals(model);

        nvl::FilenameInfo fileInfo = nvl::getFilenameInfo(filename);
        return loadModel(model, fileInfo.file);
    }
    else {
        QMessageBox::warning(this, tr("Error"), tr("Error: impossible to load model!"));
        return nvl::MAX_ID;
    }
}

nvl::Index SkinMixerManager::loadModel(Model* model, const std::string& name)
{
    ModelDrawer* modelDrawer = new ModelDrawer(model);
    modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
    modelDrawer->meshDrawer().setWireframeVisible(true);

    vModelDrawers.insert(modelDrawer);
    vModelMap.insert(std::make_pair(modelDrawer, model));

    skinMixerData.addEntry(model);

    return vCanvas->addDrawable(modelDrawer, name);
}

nvl::Index SkinMixerManager::loadModel(const Model& model, const std::string& name)
{
    Model* modelPtr = new Model(model);
    vModels.insert(modelPtr);
    return loadModel(modelPtr, name);
}

bool SkinMixerManager::removeModelDrawer(ModelDrawer* modelDrawer)
{
    std::unordered_set<ModelDrawer*>::iterator modelDrawerIt = vModelDrawers.find(modelDrawer);
    if (modelDrawerIt != vModelDrawers.end()) {
        vModelDrawers.erase(modelDrawerIt);
        bool success = vCanvas->removeDrawable(modelDrawer);
        assert(success);
        delete modelDrawer;

        std::unordered_map<ModelDrawer*, Model*>::iterator modelIt = vModelMap.find(modelDrawer);
        if (modelIt != vModelMap.end()) {
            vModelMap.erase(modelIt);
            vModels.erase(modelIt->second);
            delete modelIt->second;
        }

        return true;
    }
    else {
        QMessageBox::warning(this, tr("Error"), tr("Impossible to remove a drawable object: the object was not added to the canvas by the loader."));

        return false;
    }
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

    currentOperation = OperationType::NONE;

    updateView();
    updateCanvasView();

    updateSkinningWeightVertexValues();

    vCanvas->updateGL();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<nvl::Skeleton3d::JointId>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    currentOperation = OperationType::NONE;

    updateView();
    updateCanvasView();

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
            modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
            modelDrawer->meshDrawer().setTrasparencyEnabled(true);

            modelDrawer->skeletonDrawer().setVisible(true);
            modelDrawer->skeletonDrawer().setJointSize(8);
            modelDrawer->skeletonDrawer().setTrasparencyEnabled(true);
        }
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

void SkinMixerManager::applyOperation()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    bool drawableSelected = selectedModelDrawer != nullptr;
    bool jointSelected = drawableSelected && selectedJointId != nvl::MAX_ID;

    if (jointSelected) {
        SkinMixerEntry& entry = skinMixerData.entry(selectedModelDrawer->model());

        std::vector<float>& vertexFuzzyValue = entry.vertexFuzzyValue;
        std::vector<float>& jointFuzzyValue = entry.jointFuzzyValue;

        if (currentOperation == OperationType::REMOVE) {
            skinmixer::remove(*entry.model, selectedJointId, vertexFuzzyValue, jointFuzzyValue);
        }
        else if (currentOperation == OperationType::DETACH) {
            skinmixer::detach(*entry.model, selectedJointId, vertexFuzzyValue, jointFuzzyValue);
        }

        colorizeModelDrawerWithFuzzyValues(selectedModelDrawer);
    }

    attachFirstModelDrawer = nullptr;
    attachFirstJointId = nvl::MAX_ID;
    attachBackupFrame = nvl::Affine3d::Identity();

    currentOperation = OperationType::NONE;
}

void SkinMixerManager::updateCanvasView()
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    if (this->lastSelectedModelDrawer != nullptr) {
        colorizeModelDrawerWithFuzzyValues(lastSelectedModelDrawer);
    }

    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    bool drawableSelected = selectedModelDrawer != nullptr;
    bool jointSelected = drawableSelected && selectedJointId != nvl::MAX_ID;

    if (currentOperation != OperationType::NONE && jointSelected) {
        Model* modelPtr = selectedModelDrawer->model();

        const Model& model = *modelPtr;
        const Mesh& mesh = model.mesh;
        const Skeleton& skeleton = model.skeleton;

        if (currentOperation == OperationType::REMOVE || currentOperation == OperationType::DETACH) {
            const SkinMixerEntry& entry = skinMixerData.entry(modelPtr);

            const std::vector<float>& originalVertexFuzzyValue = entry.vertexFuzzyValue;
            const std::vector<float>& originalJointFuzzyValue = entry.jointFuzzyValue;

            std::vector<float> previewVertexFuzzyValue = originalVertexFuzzyValue;
            std::vector<float> previewJointFuzzyValue = originalJointFuzzyValue;

            if (currentOperation == OperationType::REMOVE) {
                skinmixer::remove(model, selectedJointId, previewVertexFuzzyValue, previewJointFuzzyValue);
            }
            else if (currentOperation == OperationType::DETACH) {
                skinmixer::detach(model, selectedJointId, previewVertexFuzzyValue, previewJointFuzzyValue);
            }

            colorizeModelDrawerWithFuzzyValues(selectedModelDrawer, previewVertexFuzzyValue, previewJointFuzzyValue);

            for (FaceId fId = 0; fId < mesh.nextFaceId(); fId++) {
                if (mesh.isFaceDeleted(fId))
                    continue;

                float avgValue = 0.0;

                const Face& face = mesh.face(fId);

                bool faceAffected = false;
                for (VertexId j = 0; j < face.vertexNumber(); j++) {
                    VertexId vId = face.vertexId(j);

                    if (previewVertexFuzzyValue[vId] < originalVertexFuzzyValue[vId]) {
                        faceAffected = true;
                    }

                    float alphaValue = std::max(previewVertexFuzzyValue[vId], 0.1f);
                    avgValue += alphaValue;
                }
                avgValue /= face.vertexNumber();

                if (faceAffected) {
                    nvl::Color wireframeC = selectedModelDrawer->meshDrawer().renderingFaceWireframeColor(fId);
                    wireframeC.setAlphaF(avgValue);
                    selectedModelDrawer->meshDrawer().setRenderingFaceWireframeColor(fId, wireframeC);
                }
            }

            for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
                if (mesh.isVertexDeleted(vId))
                    continue;

                if (previewVertexFuzzyValue[vId] < originalVertexFuzzyValue[vId]) {
                    float alphaValue = std::max(previewVertexFuzzyValue[vId], 0.0f);

                    nvl::Color vertexC = selectedModelDrawer->meshDrawer().renderingVertexColor(vId);
                    vertexC.setAlphaF(alphaValue);
                    selectedModelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
                }
            }

            for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
                if (previewJointFuzzyValue[jId] < originalJointFuzzyValue[jId]) {
                    float jointAlphaValue = std::max(previewJointFuzzyValue[jId], 0.1f);

                    nvl::Color jointC = selectedModelDrawer->skeletonDrawer().renderingJointColor(jId);
                    jointC.setAlphaF(jointAlphaValue);
                    selectedModelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

                    if (skeleton.parent(jId) != nvl::MAX_ID) {
                        float boneAlphaValue = std::min(std::max(previewJointFuzzyValue[skeleton.parent(jId)], 0.1f), jointAlphaValue);

                        nvl::Color boneC = selectedModelDrawer->skeletonDrawer().renderingBoneColor(jId);
                        boneC.setAlphaF(boneAlphaValue);
                        selectedModelDrawer->skeletonDrawer().setRenderingBoneColor(jId, boneC);
                    }
                }
            }
        }
    }

    if (jointSelected) {
        selectedModelDrawer->skeletonDrawer().setRenderingJointColor(selectedJointId, nvl::Color(1.0, 1.0, 0.0));
    }
    this->lastSelectedModelDrawer = selectedModelDrawer;
}

void SkinMixerManager::updateView()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    bool atLeastOneDrawableSelected = vDrawableListWidget->selectedDrawables().size() > 0;
    bool drawableSelected = selectedModelDrawer != nullptr;
    bool jointSelected = drawableSelected && selectedJointId != nvl::MAX_ID;

    ui->modelLoadButton->setEnabled(true);
    ui->modelRemoveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelSaveButton->setEnabled(atLeastOneDrawableSelected);

    ui->operationDetachButton->setEnabled(jointSelected && currentOperation == OperationType::NONE);
    ui->operationRemoveButton->setEnabled(jointSelected && currentOperation == OperationType::NONE);
    ui->operationAttachButton->setEnabled(jointSelected && currentOperation == OperationType::NONE);
    ui->operationAbortButton->setEnabled(jointSelected && currentOperation != OperationType::NONE);
    ui->operationApplyButton->setEnabled(jointSelected && currentOperation != OperationType::NONE);
}

void SkinMixerManager::colorizeModelDrawerWithFuzzyValues(
        ModelDrawer* modelDrawer)
{
    const SkinMixerEntry& entry = skinMixerData.entry(modelDrawer->model());
    colorizeModelDrawerWithFuzzyValues(
        modelDrawer,
        entry.vertexFuzzyValue,
        entry.jointFuzzyValue);
}

void SkinMixerManager::colorizeModelDrawerWithFuzzyValues(
        ModelDrawer* modelDrawer,
        const std::vector<float>& vertexFuzzyValue,
        const std::vector<float>& jointFuzzyValue)
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    modelDrawer->resetRenderingData();

    const Model& model = *modelDrawer->model();
    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    for (FaceId fId = 0; fId < mesh.nextFaceId(); fId++) {
        if (mesh.isFaceDeleted(fId))
            continue;

        float avgValue = 0.0;

        const Face& face = mesh.face(fId);
        for (VertexId j = 0; j < face.vertexNumber(); j++) {
            VertexId vId = face.vertexId(j);
            float value = vertexFuzzyValue[vId];
            avgValue += value;
        }
        avgValue /= face.vertexNumber();

        nvl::Color wireframeC = modelDrawer->meshDrawer().renderingFaceWireframeColor(fId);
        wireframeC.setAlphaF(avgValue);

        modelDrawer->meshDrawer().setRenderingFaceWireframeColor(fId, wireframeC);
    }

    for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
        if (mesh.isVertexDeleted(vId))
            continue;

        float value = vertexFuzzyValue[vId];

        nvl::Color vertexC = modelDrawer->meshDrawer().renderingVertexColor(vId);
        vertexC.setAlphaF(value);

        modelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
    }


    for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
        float jointValue = jointFuzzyValue[jId];

        nvl::Color jointC = modelDrawer->skeletonDrawer().renderingJointColor(jId);
        jointC.setAlphaF(jointValue);
        modelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

        if (skeleton.parent(jId) != nvl::MAX_ID) {
            float boneValue = std::min(jointFuzzyValue[skeleton.parent(jId)], jointValue);

            nvl::Color boneC = modelDrawer->skeletonDrawer().renderingBoneColor(jId);
            boneC.setAlphaF(boneValue);
            modelDrawer->skeletonDrawer().setRenderingBoneColor(jId, boneC);
        }
    }
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
        connect(vCanvas, &nvl::Canvas::signal_canvasPicking, this, &SkinMixerManager::slot_canvasPicking);
        connect(vCanvas, &nvl::Canvas::signal_drawableAdded, this, &SkinMixerManager::slot_drawableAdded);
    }
}

void SkinMixerManager::on_modelLoadButton_clicked()
{
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    QStringList filters;
    filters
            << "Model (*.mdl)"
            << "Any files (*)";
    dialog.setNameFilters(filters);

    QStringList files;

    if (dialog.exec()) {
        files = dialog.selectedFiles();
    }

    for (const QString& str : files) {
        loadModelFromFile(str.toStdString());
    }

    vCanvas->fitScene();
}

void SkinMixerManager::on_modelRemoveButton_clicked()
{
    size_t offset = 0;
    const std::unordered_set<Index>& selectedDrawables = vDrawableListWidget->selectedDrawables();
    while (selectedDrawables.size() > offset) {
        std::unordered_set<Index>::const_iterator it = selectedDrawables.begin();

        for (size_t i = 0; i < offset; i++) {
            it++;
        }

        Index id = *it;

        bool success = false;
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(vCanvas->drawable(id));
        if (modelDrawer != nullptr) {
            success = removeModelDrawer(modelDrawer);
        }
        if (!success) {
            offset++;
        }
    }

    vCanvas->updateGL();
}

void SkinMixerManager::on_modelSaveButton_clicked()
{
    //TODO
}

void SkinMixerManager::on_modelDuplicateButton_clicked()
{
    const std::unordered_set<Index> selectedDrawables = vDrawableListWidget->selectedDrawables();
    for (Index selected : selectedDrawables) {
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(vCanvas->drawable(selected));
        if (modelDrawer != nullptr) {
            Model* model = new Model(*modelDrawer->model());
            loadModel(model, vCanvas->drawableName(selected));
        }
    }

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetSlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateCanvasView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetResetButton_clicked()
{
    ui->cutOffsetSlider->setValue(0);
    updateCanvasView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutRigiditySlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateCanvasView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetRigidityButton_clicked()
{
    ui->cutRigiditySlider->setValue(60);

    updateCanvasView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationDetachButton_clicked()
{
    currentOperation = OperationType::DETACH;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationRemoveButton_clicked()
{
    currentOperation = OperationType::REMOVE;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAttachButton_clicked()
{
    currentOperation = OperationType::ATTACH;

    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    assert(selectedModelDrawer != nullptr && selectedJointId != nvl::MAX_ID);
    attachFirstModelDrawer = selectedModelDrawer;
    attachFirstJointId = selectedJointId;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAbortButton_clicked()
{
    currentOperation = OperationType::NONE;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationApplyButton_clicked()
{
    applyOperation();

    vCanvas->updateGL();
}

void SkinMixerManager::clear()
{
    while (!vModelDrawers.empty()) {
        ModelDrawer* drawable = *(vModelDrawers.begin());
        removeModelDrawer(drawable);
    }
}

void SkinMixerManager::updateModelNormals(Model* model)
{
    if (ui->updateFaceNormalsCheckBox->isChecked()) {
        nvl::meshUpdateFaceNormals(model->mesh);
    }
    if (ui->updateVertexNormalsCheckBox->isChecked()) {
        nvl::meshUpdateVertexNormals(model->mesh);
    }
}
