#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/timer.h>
#include <nvl/utilities/colorize.h>

#include <nvl/models/model_io.h>

#include <QFileDialog>
#include <QMessageBox>

#include <iostream>

#include "skinmixer/skinmixer.h"

#include <nvl/models/mesh_normals.h>

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
    vCurrentOperation(OperationType::NONE),
    vSelectedModelDrawer(nullptr),
    vSelectedJoint(nvl::MAX_ID),
    vAttachModelDrawer(nullptr),
    vAttachJoint(nvl::MAX_ID),
    vBackupFrame(nvl::Affine3d::Identity())
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
        initializeLoadedModel(model);

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

    vSkinMixerData.addEntry(model);

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

    if (vSelectedModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vAttachModelDrawer);
    }

    vSelectedModelDrawer = getSelectedModelDrawer();
    vSelectedJoint = getSelectedJointId();

    if (vCurrentOperation == OperationType::ATTACH) {
        assert(vAttachModelDrawer != nullptr);
        prepareModelForAttach();
    }
    else {
        abortOperation();
    }

    updateView();
    updateCanvasView();

    vCanvas->updateGL();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<Index>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    if (vSelectedModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vAttachModelDrawer);
    }

    vSelectedModelDrawer = getSelectedModelDrawer();
    vSelectedJoint = getSelectedJointId();

    if (vCurrentOperation == OperationType::ATTACH) {
        assert(vAttachModelDrawer != nullptr);
        prepareModelForAttach();
    }
    else {
        abortOperation();
    }

    updateView();
    updateCanvasView();

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    vCanvas->updateGL();
}

void SkinMixerManager::slot_movableFrameChanged()
{
    nvl::Affine3d transform = vCanvas->movableFrame();

    nvl::Affine3d::LinearMatrixType scaMatrix;
    transform.computeScalingRotation(&scaMatrix, static_cast<nvl::Affine3d::LinearMatrixType*>(0));
    nvl::Scaling3d sca(scaMatrix.diagonal());

    nvl::Quaterniond rot(transform.rotation());

    nvl::Translation3d tra(transform.translation());

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

            //Scale
            newFrame = sca * newFrame;

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
    vCanvas->updateGL();
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

void SkinMixerManager::mix()
{
    std::vector<nvl::Index> newEntries = skinmixer::mix(vSkinMixerData);

    for (nvl::Index eId : newEntries) {
        SkinMixerEntry& newEntry = vSkinMixerData.entry(eId);

        Model* newModel = newEntry.model;
        nvl::meshUpdateFaceNormals(newModel->mesh);
        nvl::meshUpdateVertexNormals(newModel->mesh);

        ModelDrawer* modelDrawer = new ModelDrawer(newModel);
        modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
        modelDrawer->meshDrawer().setWireframeVisible(true);
        modelDrawer->meshDrawer().setTrasparencyEnabled(false);
        modelDrawer->skeletonDrawer().setTrasparencyEnabled(false);

        vModelDrawers.insert(modelDrawer);
        vModelMap.insert(std::make_pair(modelDrawer, newModel));

        vCanvas->addDrawable(modelDrawer, "Result");
    }
}

void SkinMixerManager::applyOperation()
{
    if (vCurrentOperation == OperationType::REMOVE || vCurrentOperation == OperationType::DETACH) {
        assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_ID);

        const unsigned int functionSmoothingIterations = ui->functionSmoothingSlider->value();
        const double offset = ui->cutOffsetSlider->value() / 50.0;
        const double rigidity = ui->cutRigiditySlider->value() / 100.0;

        if (vCurrentOperation == OperationType::REMOVE) {
            skinmixer::remove(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, functionSmoothingIterations, offset, rigidity);
        }
        else if (vCurrentOperation == OperationType::DETACH) {
            skinmixer::detach(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, functionSmoothingIterations, offset, rigidity);
        }

        if (vSelectedModelDrawer != nullptr) {
            colorizeModelDrawerWithSelectValues(vSelectedModelDrawer);
        }
    }
    else if (vCurrentOperation == OperationType::ATTACH) {
        assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_ID);
        assert(vAttachModelDrawer != nullptr && vAttachJoint != nvl::MAX_ID);

        nvl::Affine3d transformation1 = vAttachModelDrawer->frame();
        nvl::Affine3d transformation2 = vSelectedModelDrawer->frame();

        skinmixer::attach(vSkinMixerData, vAttachModelDrawer->model(), vSelectedModelDrawer->model(), vAttachJoint, vSelectedJoint, transformation1, transformation2);

        vAttachModelDrawer->resetFrame();
        vSelectedModelDrawer->resetFrame();

        colorizeModelDrawerWithSelectValues(vAttachModelDrawer);
        colorizeModelDrawerWithSelectValues(vSelectedModelDrawer);
    }

    vCurrentOperation = OperationType::NONE;

    vAttachModelDrawer = nullptr;
    vAttachJoint = nvl::MAX_ID;
    vBackupFrame = nvl::Affine3d::Identity();
}

void SkinMixerManager::abortOperation()
{
    if (vCurrentOperation == OperationType::ATTACH && vSelectedModelDrawer != vAttachModelDrawer && vSelectedModelDrawer != nullptr) {
        vSelectedModelDrawer->setFrame(vBackupFrame);
    }

    vCurrentOperation = OperationType::NONE;

    if (vSelectedModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeModelDrawerWithSelectValues(vAttachModelDrawer);
    }

    vAttachModelDrawer = nullptr;
    vAttachJoint = nvl::MAX_ID;
    vBackupFrame = nvl::Affine3d::Identity();
}

void SkinMixerManager::updateCanvasView()
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;


    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_ID;

    bool attachDrawableSelected = vAttachModelDrawer != nullptr;
    bool attachJointSelected = attachDrawableSelected && vAttachJoint != nvl::MAX_ID;

    if (vCurrentOperation != OperationType::NONE && jointSelected) {
        Model* modelPtr = vSelectedModelDrawer->model();

        const Model& model = *modelPtr;
        const Mesh& mesh = model.mesh;
        const Skeleton& skeleton = model.skeleton;

        if (vCurrentOperation == OperationType::REMOVE || vCurrentOperation == OperationType::DETACH) {
            const unsigned int functionSmoothingIterations = ui->functionSmoothingSlider->value();
            const double offset = ui->cutOffsetSlider->value() / 50.0;
            const double rigidity = ui->cutRigiditySlider->value() / 100.0;

            const SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

            const std::vector<float>& originalVertexSelectValue = entry.select.vertex;
            const std::vector<bool>& originalJointSelectValue = entry.select.joint;

            std::vector<float> previewVertexSelectValue = originalVertexSelectValue;
            std::vector<bool> previewJointSelectValue = originalJointSelectValue;

            if (vCurrentOperation == OperationType::REMOVE) {
                skinmixer::computeRemoveSelectValues(model, vSelectedJoint, functionSmoothingIterations, offset, rigidity, previewVertexSelectValue, previewJointSelectValue);
            }
            else if (vCurrentOperation == OperationType::DETACH) {
                skinmixer::computeDetachSelectValues(model, vSelectedJoint, functionSmoothingIterations, offset, rigidity, previewVertexSelectValue, previewJointSelectValue);
            }

            colorizeModelDrawerWithSelectValues(vSelectedModelDrawer, previewVertexSelectValue, previewJointSelectValue);

            for (FaceId fId = 0; fId < mesh.nextFaceId(); fId++) {
                if (mesh.isFaceDeleted(fId))
                    continue;

                float avgValue = 0.0;

                const Face& face = mesh.face(fId);

                bool faceAffected = false;
                for (VertexId j = 0; j < face.vertexNumber(); j++) {
                    VertexId vId = face.vertexId(j);

                    if (vId >= originalVertexSelectValue.size() || previewVertexSelectValue[vId] < originalVertexSelectValue[vId]) {
                        faceAffected = true;
                    }

                    float alphaValue = std::max(std::min(previewVertexSelectValue[vId], 1.0f), 0.1f);
                    avgValue += alphaValue;
                }
                avgValue /= face.vertexNumber();

                if (faceAffected) {
                    nvl::Color wireframeC = vSelectedModelDrawer->meshDrawer().renderingFaceWireframeColor(fId);
                    wireframeC.setAlphaF(avgValue);
                    vSelectedModelDrawer->meshDrawer().setRenderingFaceWireframeColor(fId, wireframeC);
                }
            }

            for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
                if (mesh.isVertexDeleted(vId))
                    continue;

                if (vId >= originalVertexSelectValue.size() || previewVertexSelectValue[vId] < originalVertexSelectValue[vId]) {
                    float alphaValue = std::max(std::min(previewVertexSelectValue[vId], 1.0f), 0.1f);

                    nvl::Color vertexC = vSelectedModelDrawer->meshDrawer().renderingVertexColor(vId);
                    vertexC.setAlphaF(alphaValue);
                    vSelectedModelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
                }
            }

            for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {

                if (jId >= originalJointSelectValue.size() || previewJointSelectValue[jId] != originalJointSelectValue[jId]) {
                    float jointAlphaValue = (jId >= originalJointSelectValue.size() || originalJointSelectValue[jId]) ? (previewJointSelectValue[jId] ?  1.0f : 0.1f) : 0.0;

                    nvl::Color jointC = vSelectedModelDrawer->skeletonDrawer().renderingJointColor(jId);
                    jointC.setAlphaF(jointAlphaValue);
                    vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

                }
                if (!skeleton.isRoot(jId)) {
                    JointId parentJointId = skeleton.parent(jId);

                    if (jId >= originalJointSelectValue.size() || parentJointId >= originalJointSelectValue.size() || previewJointSelectValue[jId] != originalJointSelectValue[jId] || previewJointSelectValue[parentJointId] != originalJointSelectValue[parentJointId]) {
                        JointId parentJointId = skeleton.parent(jId);
                        float boneAlphaValue = (jId >= originalJointSelectValue.size() || parentJointId >= originalJointSelectValue.size() || (previewJointSelectValue[parentJointId] && previewJointSelectValue[jId]) ?  1.0f : 0.1f);

                        nvl::Color boneC = vSelectedModelDrawer->skeletonDrawer().renderingBoneColor(jId);
                        boneC.setAlphaF(boneAlphaValue);
                        vSelectedModelDrawer->skeletonDrawer().setRenderingBoneColor(jId, boneC);
                    }
                }
            }

        }
    }

    if (jointSelected) {
        vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vSelectedJoint, nvl::Color(1.0, 1.0, 0.0));
    }
    if (attachJointSelected) {
        vAttachModelDrawer->skeletonDrawer().setRenderingJointColor(vAttachJoint, nvl::Color(0.0, 1.0, 1.0));
    }
}

void SkinMixerManager::prepareModelForAttach()
{
    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_ID;

    if (jointSelected) {
        if (vSelectedModelDrawer != vAttachModelDrawer) {
            typedef typename Model::Mesh::Point Point;
            typedef typename Model::Skeleton Skeleton;
            typedef typename Skeleton::JointId JointId;

            const Model* model1 = vAttachModelDrawer->model();
            const Skeleton& skeleton1 = model1->skeleton;
            const JointId targetJoint1 = vAttachJoint;

            const Model* model2 = vSelectedModelDrawer->model();
            const Skeleton& skeleton2 = model2->skeleton;
            const JointId targetJoint2 = vSelectedJoint;

            vBackupFrame = vSelectedModelDrawer->frame();

            Point v1 = vAttachModelDrawer->frame() * (skeleton1.joint(targetJoint1).restTransform() * Point(0,0,0));
            Point v2 = vSelectedModelDrawer->frame() * (skeleton2.joint(targetJoint2).restTransform() * Point(0,0,0));
            Point translate = v1 - v2;

            nvl::Affine3d transform = nvl::getTranslationAffine3(translate);

            vSelectedModelDrawer->setFrame(transform * vSelectedModelDrawer->frame());
        }
        else {
            abortOperation();
        }
    }
}

void SkinMixerManager::updateView()
{
    bool atLeastOneDrawableSelected = vDrawableListWidget->selectedDrawables().size() > 0;
    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_ID;

    ui->modelLoadButton->setEnabled(true);
    ui->modelRemoveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelSaveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelDuplicateButton->setEnabled(atLeastOneDrawableSelected);

    ui->operationDetachButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationRemoveButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationAttachButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationAbortButton->setEnabled(jointSelected && vCurrentOperation != OperationType::NONE);
    ui->operationApplyButton->setEnabled(jointSelected && vCurrentOperation != OperationType::NONE);
    ui->mixButton->setEnabled(vCurrentOperation == OperationType::NONE && !vSkinMixerData.actions().empty());
    ui->updateValuesWeightsButton->setEnabled(jointSelected);
    ui->updateValuesBirthButton->setEnabled(modelDrawerSelected);
}

void SkinMixerManager::colorizeModelDrawerWithSelectValues(
        ModelDrawer* modelDrawer)
{
    const SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelDrawer->model());

    colorizeModelDrawerWithSelectValues(
        modelDrawer,
        entry.select.vertex,
        entry.select.joint);
}

void SkinMixerManager::colorizeModelDrawerWithSelectValues(
        ModelDrawer* modelDrawer,
        const std::vector<float>& vertexSelectValue,
        const std::vector<bool>& jointSelectValue)
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    //Reset of the model drawer
    modelDrawer->update();

    if (vertexSelectValue.empty() || jointSelectValue.empty()) {
        return;
    }

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
            float value = std::max(std::min(vertexSelectValue[vId], 1.0f), 0.0f);
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

        float value = std::max(std::min(vertexSelectValue[vId], 1.0f), 0.0f);;

        nvl::Color vertexC = modelDrawer->meshDrawer().renderingVertexColor(vId);
        vertexC.setAlphaF(value);

        modelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
    }


    for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
        float jointValue = jointSelectValue[jId] ? 1.0 : 0.0;

        nvl::Color jointC = modelDrawer->skeletonDrawer().renderingJointColor(jId);
        jointC.setAlphaF(jointValue);
        modelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

        if (!skeleton.isRoot(jId)) {
            JointId parentJointId = skeleton.parent(jId);
            float boneValue = jointSelectValue[jId] && jointSelectValue[parentJointId] ? 1.0 : 0.0;

            nvl::Color boneC = modelDrawer->skeletonDrawer().renderingBoneColor(jId);
            boneC.setAlphaF(boneValue);
            modelDrawer->skeletonDrawer().setRenderingBoneColor(jId, boneC);
        }
    }
}


void SkinMixerManager::updateValuesReset()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();

    if (selectedModelDrawer != nullptr) {
        selectedModelDrawer->meshDrawer().clearVertexValues();

        vCanvas->updateGL();
    }
}

void SkinMixerManager::updateValuesSkinningWeights()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    JointId selectedJointId = getSelectedJointId();

    if (selectedModelDrawer != nullptr) {
        std::vector<double> vertexValues;

        if (selectedJointId != nvl::MAX_ID) {
            vertexValues.resize(selectedModelDrawer->model()->mesh.nextVertexId(), 0.0);

            for (auto vertex : selectedModelDrawer->model()->mesh.vertices()) {
                vertexValues[vertex.id()] = selectedModelDrawer->model()->skinningWeights.weight(vertex.id(), selectedJointId);
            }
        }

        selectedModelDrawer->meshDrawer().setVertexValues(vertexValues);

        vCanvas->updateGL();
    }
}

void SkinMixerManager::updateValuesBirth()
{
    ModelDrawer* selectedModelDrawer = getSelectedModelDrawer();
    if (selectedModelDrawer != nullptr) {
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(selectedModelDrawer->model());

        if (!entry.birth.vertex.empty()) {
            std::set<nvl::Index> birthEntries;
            for (auto vertex : selectedModelDrawer->model()->mesh.vertices()) {
                for (auto info : entry.birth.vertex[vertex.id()]) {
                    birthEntries.insert(info.eId);
                }
            }

            if (birthEntries.size() > 2) {
                selectedModelDrawer->meshDrawer().clearVertexValues();
            }
            else {
                nvl::Index firstEntry = *birthEntries.begin();

                std::vector<double> vertexValues;
                vertexValues.resize(selectedModelDrawer->model()->mesh.nextVertexId(), -1.0);

                for (auto vertex : selectedModelDrawer->model()->mesh.vertices()) {
                    double currentValue = 0.0;

                    nvl::Size n = 0;

                    for (auto info : entry.birth.vertex[vertex.id()]) {
                        if (info.vId == nvl::MAX_ID) {
                            if (info.eId == firstEntry) {
                                currentValue += 1 - info.selectValue;
                            }
                            else {
                                currentValue += info.selectValue;
                            }
                            n++;
                        }
                        else {
                            currentValue = -1;
                            n = 1;
                            break;
                        }
                    }

                    currentValue /= n;

                    vertexValues[vertex.id()] = currentValue;
                }

                selectedModelDrawer->meshDrawer().setVertexValues(vertexValues);
            }


            vCanvas->updateGL();
        }
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
    vCurrentOperation = OperationType::DETACH;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationRemoveButton_clicked()
{
    vCurrentOperation = OperationType::REMOVE;

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAttachButton_clicked()
{
    vCurrentOperation = OperationType::ATTACH;

    assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_ID);
    vAttachModelDrawer = vSelectedModelDrawer;
    vAttachJoint = vSelectedJoint;
    vBackupFrame = nvl::Affine3d::Identity();

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAbortButton_clicked()
{
    abortOperation();

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationApplyButton_clicked()
{
    applyOperation();

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::clear()
{
    while (!vModelDrawers.empty()) {
        ModelDrawer* drawable = *(vModelDrawers.begin());
        removeModelDrawer(drawable);
    }
}

void SkinMixerManager::initializeLoadedModel(Model* model)
{
    if (ui->scaleOn1AndCenterCheckBox->isChecked()) {
        nvl::AlignedBox3d bbox = nvl::meshBoundingBox(model->mesh);
        double scaleFactor = 1.0 / bbox.diagonal().norm();
        nvl::Point3d translateVector = -bbox.center();

        nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);
        nvl::Translation3d translateTransform(translateVector);

        nvl::Affine3d transformation(scaleTransform * translateTransform);
        nvl::modelApplyTransformation(*model, transformation);
    }
    if (ui->updateFaceNormalsCheckBox->isChecked()) {
        nvl::meshUpdateFaceNormals(model->mesh);
    }
    if (ui->updateVertexNormalsCheckBox->isChecked()) {
        nvl::meshUpdateVertexNormals(model->mesh);
    }
}

void SkinMixerManager::on_mixButton_clicked()
{
    mix();

    updateCanvasView();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_updateValuesResetButton_clicked()
{
    updateValuesReset();
}

void SkinMixerManager::on_updateValuesWeightsButton_clicked()
{
    updateValuesSkinningWeights();
}

void SkinMixerManager::on_updateValuesBirthButton_clicked()
{
    updateValuesBirth();
}
