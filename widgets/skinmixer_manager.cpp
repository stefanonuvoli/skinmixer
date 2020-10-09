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
        nvl::ModelAnimationWidget* modelAnimationWidget,
        QWidget *parent) :
    QFrame(parent),
    ui(new Ui::SkinMixerManager),
    vCanvas(canvas),
    vDrawableListWidget(drawableListWidget),
    vSkeletonJointListWidget(skeletonJointListWidget),
    vModelAnimationWidget(modelAnimationWidget),
    vCurrentOperation(OperationType::NONE),
    vSelectedModelDrawer(nullptr),
    vSelectedJoint(nvl::MAX_INDEX),
    vAttachModelDrawer(nullptr),
    vAttachJoint(nvl::MAX_INDEX),
    vBackupFrame(nvl::Affine3d::Identity()),
    vBlendingAnimation(nvl::MAX_INDEX)
{
    ui->setupUi(this);

    initialize();
    connectSignals();
}

SkinMixerManager::~SkinMixerManager()
{
    for (std::pair<Model*, ModelDrawer*> entry : vModelToDrawerMap) {
        delete entry.second;
    }

    for (Model* model : vModels) {
        delete model;
    }
    vModels.clear();

    vDrawerToModelMap.clear();
    vModelToDrawerMap.clear();

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
        return loadModel(model);
    }
    else {
        QMessageBox::warning(this, tr("Error"), tr("Error: impossible to load model!"));
        return nvl::MAX_INDEX;
    }
}

nvl::Index SkinMixerManager::loadModel(Model* model)
{
    ModelDrawer* modelDrawer = new ModelDrawer(model);
    modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
    modelDrawer->meshDrawer().setWireframeVisible(true);

    vDrawerToModelMap.insert(std::make_pair(modelDrawer, model));
    vModelToDrawerMap.insert(std::make_pair(model, modelDrawer));

    vSkinMixerData.addEntry(model);

    return vCanvas->addDrawable(modelDrawer, model->name());
}

nvl::Index SkinMixerManager::loadModel(const Model& model)
{
    Model* modelPtr = new Model(model);
    vModels.insert(modelPtr);
    return loadModel(modelPtr);
}

bool SkinMixerManager::removeModelDrawer(ModelDrawer* modelDrawer)
{
    std::unordered_map<ModelDrawer*, Model*>::iterator drawerIt = vDrawerToModelMap.find(modelDrawer);
    if (drawerIt != vDrawerToModelMap.end()) {
        std::unordered_map<Model*, ModelDrawer*>::iterator modelIt = vModelToDrawerMap.find(drawerIt->second);
        if (modelIt != vModelToDrawerMap.end()) {
            vModelToDrawerMap.erase(modelIt);
        }

        vDrawerToModelMap.erase(drawerIt);

        vCanvas->removeDrawable(modelDrawer);
        delete modelDrawer;

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
        colorizeByData(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeByData(vAttachModelDrawer);
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
    updateCanvasPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<Index>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    if (vBlendingAnimation != nvl::MAX_INDEX) {
        vSelectedModelDrawer->unloadAnimation();
        vSelectedModelDrawer->model()->removeAnimation(vBlendingAnimation);
        vBlendingAnimation = nvl::MAX_INDEX;
    }

    if (vSelectedModelDrawer != nullptr) {
        colorizeByData(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeByData(vAttachModelDrawer);
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
    updateCanvasPreview();

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
            modelDrawer->meshDrawer().setSurfaceTransparency(true);

            modelDrawer->skeletonDrawer().setVisible(true);
            modelDrawer->skeletonDrawer().setJointSize(8);
            modelDrawer->skeletonDrawer().setTransparency(true);
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
    SkinMixerManager::JointId selectedJointId = nvl::MAX_INDEX;

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

        vDrawerToModelMap.insert(std::make_pair(modelDrawer, newModel));
        vModelToDrawerMap.insert(std::make_pair(newModel, modelDrawer));

        for (const Index& bId : newEntry.birth.entries) {
            Model* model = vSkinMixerData.entry(bId).model;
            typename std::unordered_map<Model*, ModelDrawer*>::iterator it = vModelToDrawerMap.find(model);
            if (it != vModelToDrawerMap.end()) {
                //TODO
//                removeModelDrawer(it->second);
                it->second->setVisible(false);
            }
        }

        vCanvas->addDrawable(modelDrawer, "Result");
    }
}

void SkinMixerManager::blendAnimations()
{
    Model* modelPtr = vSelectedModelDrawer->model();
    SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

    vBlendingAnimation = vModelAnimationWidget->selectedAnimation();

    skinmixer::mixAnimations(vSkinMixerData, entry, vBlendingAnimation);

    vModelAnimationWidget->selectAnimation(vBlendingAnimation);
    vCanvas->stopAnimations();
    vCanvas->updateGL();
}

void SkinMixerManager::applyOperation()
{
    if (vCurrentOperation == OperationType::REMOVE || vCurrentOperation == OperationType::DETACH) {
        assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_INDEX);

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
            colorizeByData(vSelectedModelDrawer);
        }
    }
    else if (vCurrentOperation == OperationType::ATTACH) {
        assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_INDEX);
        assert(vAttachModelDrawer != nullptr && vAttachJoint != nvl::MAX_INDEX);

        nvl::Affine3d transformation1 = vAttachModelDrawer->frame();
        nvl::Affine3d transformation2 = vSelectedModelDrawer->frame();

        skinmixer::attach(vSkinMixerData, vAttachModelDrawer->model(), vSelectedModelDrawer->model(), vAttachJoint, vSelectedJoint, transformation1, transformation2);

        vAttachModelDrawer->resetFrame();
        vSelectedModelDrawer->resetFrame();

        colorizeByData(vAttachModelDrawer);
        colorizeByData(vSelectedModelDrawer);
    }

    vCurrentOperation = OperationType::NONE;

    vAttachModelDrawer = nullptr;
    vAttachJoint = nvl::MAX_INDEX;
    vBackupFrame = nvl::Affine3d::Identity();
}

void SkinMixerManager::abortOperation()
{
    if (vCurrentOperation == OperationType::ATTACH && vSelectedModelDrawer != vAttachModelDrawer && vSelectedModelDrawer != nullptr) {
        vSelectedModelDrawer->setFrame(vBackupFrame);
    }

    vCurrentOperation = OperationType::NONE;

    if (vSelectedModelDrawer != nullptr) {
        colorizeByData(vSelectedModelDrawer);
    }
    if (vAttachModelDrawer != nullptr) {
        colorizeByData(vAttachModelDrawer);
    }

    vAttachModelDrawer = nullptr;
    vAttachJoint = nvl::MAX_INDEX;
    vBackupFrame = nvl::Affine3d::Identity();
}

void SkinMixerManager::updateCanvasPreview()
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_INDEX;

    bool attachDrawableSelected = vAttachModelDrawer != nullptr;
    bool attachJointSelected = attachDrawableSelected && vAttachJoint != nvl::MAX_INDEX;

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

            const std::vector<double>& originalVertexSelectValue = entry.select.vertex;
            const std::vector<bool>& originalJointSelectValue = entry.select.joint;

            std::vector<double> previewVertexSelectValue = originalVertexSelectValue;
            std::vector<bool> previewJointSelectValue = originalJointSelectValue;

            if (vCurrentOperation == OperationType::REMOVE) {
                skinmixer::computeRemoveSelectValues(model, vSelectedJoint, functionSmoothingIterations, offset, rigidity, previewVertexSelectValue, previewJointSelectValue);
            }
            else if (vCurrentOperation == OperationType::DETACH) {
                skinmixer::computeDetachSelectValues(model, vSelectedJoint, functionSmoothingIterations, offset, rigidity, previewVertexSelectValue, previewJointSelectValue);
            }

            colorizeBySelectValues(vSelectedModelDrawer, previewVertexSelectValue, previewJointSelectValue);

            for (FaceId fId = 0; fId < mesh.nextFaceId(); fId++) {
                if (mesh.isFaceDeleted(fId))
                    continue;

                double avgValue = 0.0;

                const Face& face = mesh.face(fId);

                bool faceAffected = false;
                for (VertexId j = 0; j < face.vertexNumber(); j++) {
                    VertexId vId = face.vertexId(j);

                    if (vId >= originalVertexSelectValue.size() || previewVertexSelectValue[vId] < originalVertexSelectValue[vId]) {
                        faceAffected = true;
                    }

                    double alphaValue = std::max(std::min(previewVertexSelectValue[vId], 1.0), 0.1);
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
                    double alphaValue = std::max(std::min(previewVertexSelectValue[vId], 1.0), 0.1);

                    nvl::Color vertexC = vSelectedModelDrawer->meshDrawer().renderingVertexColor(vId);
                    vertexC.setAlphaF(alphaValue);
                    vSelectedModelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
                }
            }

            for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {

                if (jId >= originalJointSelectValue.size() || previewJointSelectValue[jId] != originalJointSelectValue[jId]) {
                    double jointAlphaValue = (jId >= originalJointSelectValue.size() || originalJointSelectValue[jId]) ? (previewJointSelectValue[jId] ?  1.0 : 0.1) : 0.0;

                    nvl::Color jointC = vSelectedModelDrawer->skeletonDrawer().renderingJointColor(jId);
                    jointC.setAlphaF(jointAlphaValue);
                    vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

                }
                if (!skeleton.isRoot(jId)) {
                    JointId parentJointId = skeleton.parentId(jId);

                    if (jId >= originalJointSelectValue.size() || parentJointId >= originalJointSelectValue.size() || previewJointSelectValue[jId] != originalJointSelectValue[jId] || previewJointSelectValue[parentJointId] != originalJointSelectValue[parentJointId]) {
                        JointId parentJointId = skeleton.parentId(jId);
                        double boneAlphaValue = (jId >= originalJointSelectValue.size() || parentJointId >= originalJointSelectValue.size() || (previewJointSelectValue[parentJointId] && previewJointSelectValue[jId]) ?  1.0 : 0.1);

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
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_INDEX;

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
            Point translateVector = v1 - v2;

            nvl::Translation3d translateTransform(translateVector);
            nvl::Affine3d transform(translateTransform);

            vSelectedModelDrawer->setFrame(transform * vSelectedModelDrawer->frame());
        }
        else {
            abortOperation();
        }
    }
}

void SkinMixerManager::updateView()
{
    typedef Model::Animation Animation;

    bool atLeastOneDrawableSelected = vDrawableListWidget->selectedDrawables().size() > 0;
    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::MAX_INDEX;
    bool blendedModelSelected = modelDrawerSelected && !vSkinMixerData.entryFromModel(vSelectedModelDrawer->model()).birth.entries.empty();
    bool animationBlending = blendedModelSelected && vBlendingAnimation != nvl::MAX_INDEX;

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

    ui->animationBlendingFrame->setEnabled(blendedModelSelected);
    ui->animationSelectGroupBox->setEnabled(blendedModelSelected && !animationBlending);
    ui->animationBlendButton->setEnabled(blendedModelSelected && !animationBlending);
    ui->animationConfirmButton->setEnabled(blendedModelSelected && animationBlending);
    ui->animationAbortButton->setEnabled(blendedModelSelected && animationBlending);

    ui->animationJointFrame->setEnabled(blendedModelSelected && animationBlending && jointSelected);
    ui->animationJointGroupBox->setEnabled(blendedModelSelected && animationBlending && jointSelected);

    clearLayout(ui->animationSelectGroupBox->layout());
    clearLayout(ui->animationJointGroupBox->layout());
    ui->animationJointMeshComboBox->clear();

    if (blendedModelSelected) {
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(vSelectedModelDrawer->model());

        const std::vector<Index>& birthEntries = entry.birth.entries;

        std::vector<Index>& animationIds = entry.blendingAnimations;

        for (Index cId = 0; cId < birthEntries.size(); ++cId) {
            const Index& eId = birthEntries[cId];
            Model* currentModel = vSkinMixerData.entry(eId).model;

            QComboBox* combo = new QComboBox(this);
            combo->addItem("None");

            for (Index aId = 0; aId < currentModel->animationNumber(); aId++) {
                const Animation& animation = currentModel->animation(aId);
                combo->addItem(animation.name().c_str());
            }
            combo->setCurrentIndex(animationIds[cId] + 1);

            QComboBox::connect(combo, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            [&animationIds, cId, this](int index) {
                if (index == 0) {
                    animationIds[cId] = nvl::MAX_INDEX;
                }
                else {
                    animationIds[cId] = index - 1;
                }

                blendAnimations();
            });

            ui->animationSelectGroupBox->layout()->addWidget(combo);
        }


        if (jointSelected) {
            std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

            animationWeightSliders.resize(birthEntries.size());
            animationWeightLabels.resize(birthEntries.size());
            for (Index cId = 0; cId < birthEntries.size(); ++cId) {
                const Index& eId = birthEntries[cId];
                Model* currentModel = vSkinMixerData.entry(eId).model;

                QLabel* label = new QLabel(currentModel->name().c_str());
                label->setAlignment(Qt::AlignCenter);

                QSlider* slider = new QSlider(Qt::Orientation::Horizontal, this);
                slider->setMinimum(0);
                slider->setMaximum(100);
                slider->setValue(std::round(animationWeights[vSelectedJoint][cId] * 100.0));
                animationWeightSliders[cId] = slider;

                std::ostringstream streamObj;
                streamObj << std::fixed << std::setprecision(2) << animationWeights[vSelectedJoint][cId];
                QLabel* valueLabel = new QLabel(streamObj.str().c_str());
                valueLabel->setAlignment(Qt::AlignCenter);
                animationWeightLabels[cId] = valueLabel;

                QFrame* frame = new QFrame();
                QVBoxLayout* verticalLayout = new QVBoxLayout(nullptr);
                frame->setLayout(verticalLayout);

                verticalLayout->addWidget(label);
                verticalLayout->addWidget(slider);
                verticalLayout->addWidget(valueLabel);

                ui->animationJointGroupBox->layout()->addWidget(frame);
            }

            for (Index cId = 0; cId < animationWeightSliders.size(); ++cId) {
                QSlider* slider = animationWeightSliders[cId];
                void (QSlider:: *signal)(int) = static_cast<void (QSlider::*)(int)>(&QSlider::valueChanged);
                QSlider::connect(slider, signal,
                [&animationWeights, cId, entry, this](int value) {
                    double currentValue = value / 100.0;

                    double currentOtherSum = 0.0;
                    for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                        if (i != cId) {
                            currentOtherSum += animationWeights[vSelectedJoint][i];
                        }
                    }

                    animationWeights[vSelectedJoint][cId] = currentValue;

                    for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                        if (i != cId) {
                            double weight = 1.0 / (animationWeights[vSelectedJoint].size() - 1);
                            if (!nvl::epsEqual(currentOtherSum, 0.0)) {
                                weight = animationWeights[vSelectedJoint][i] / currentOtherSum;
                            }
                            animationWeights[vSelectedJoint][i] = (1.0 - currentValue) * weight;
                        }
                    }
                    nvl::normalize(animationWeights[vSelectedJoint]);

                    for (Index i = 0; i < animationWeightSliders.size(); ++i) {
                        QSlider* s = animationWeightSliders[i];
                        QLabel* l = animationWeightLabels[i];

                        if (i != cId) {
                            s->blockSignals(true);

                            s->setValue(std::round(animationWeights[vSelectedJoint][i] * 100.0));

                            s->blockSignals(false);
                        }

                        std::ostringstream streamObj;
                        streamObj << std::fixed << std::setprecision(2) << animationWeights[vSelectedJoint][i];
                        l->setText(streamObj.str().c_str());
                    }

                    if (ui->animationJointAllCheckBox->isChecked()) {
                        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
                            if (jId != vSelectedJoint) {
                                for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                                    animationWeights[jId][i] = animationWeights[vSelectedJoint][i];
                                }
                            }
                        }
                    }
                    else if (ui->animationJointMeshComboBox->currentIndex() > 0) {
                        Index selectedIndex = static_cast<Index>(ui->animationJointMeshComboBox->currentIndex() - 1);
                        std::vector<Index> clusterMap = nvl::getInverseMap(entry.birth.entries);
                        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
                            if (jId != vSelectedJoint) {
                                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                                for (JointInfo jointInfo : jointInfos) {
                                    assert(jointInfo.jId != nvl::MAX_INDEX);
                                    assert(jointInfo.eId != nvl::MAX_INDEX);
                                    assert(entriesMap[jointInfo.eId] != nvl::MAX_INDEX);

                                    const Index& cId = clusterMap[jointInfo.eId];

                                    if (cId == selectedIndex && jointInfo.confidence == 1.0) {
                                        for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                                            animationWeights[jId][i] = animationWeights[vSelectedJoint][i];
                                        }

                                        break;
                                    }
                                }
                            }
                        }
                    }

                    blendAnimations();
                });
            }

            ui->animationJointMeshComboBox->addItem("None");

            for (const Index& eId : birthEntries) {
                Model* currentModel = vSkinMixerData.entry(eId).model;
                ui->animationJointMeshComboBox->addItem(currentModel->name().c_str());
            }
        }
    }
}

void SkinMixerManager::colorizeByData(
        ModelDrawer* modelDrawer)
{
    const SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelDrawer->model());

    //Reset of the model drawer
    modelDrawer->update();

    colorizeBySelectValues(
        modelDrawer,
        entry.select.vertex,
        entry.select.joint);
    colorizeByAnimationWeights(
        modelDrawer,
        entry.blendingAnimationWeights);
}

void SkinMixerManager::colorizeBySelectValues(
        ModelDrawer* modelDrawer,
        const std::vector<double>& vertexSelectValue,
        const std::vector<bool>& jointSelectValue)
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    if (vertexSelectValue.empty() || jointSelectValue.empty()) {
        return;
    }

    const Model& model = *modelDrawer->model();
    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    for (FaceId fId = 0; fId < mesh.nextFaceId(); fId++) {
        if (mesh.isFaceDeleted(fId))
            continue;

        double avgValue = 0.0;

        const Face& face = mesh.face(fId);
        for (VertexId j = 0; j < face.vertexNumber(); j++) {
            VertexId vId = face.vertexId(j);
            double value = std::max(std::min(vertexSelectValue[vId], 1.0), 0.0);
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

        double value = std::max(std::min(vertexSelectValue[vId], 1.0), 0.0);;

        nvl::Color vertexC = modelDrawer->meshDrawer().renderingVertexColor(vId);
        vertexC.setAlphaF(value);

        modelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
    }


    for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
        double jointValue = jointSelectValue[jId] ? 1.0 : 0.0;

        nvl::Color jointC = modelDrawer->skeletonDrawer().renderingJointColor(jId);
        jointC.setAlphaF(jointValue);
        modelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

        if (!skeleton.isRoot(jId)) {
            JointId parentJointId = skeleton.parentId(jId);
            double boneValue = jointSelectValue[jId] && jointSelectValue[parentJointId] ? 1.0 : 0.0;

            nvl::Color boneC = modelDrawer->skeletonDrawer().renderingBoneColor(jId);
            boneC.setAlphaF(boneValue);
            modelDrawer->skeletonDrawer().setRenderingBoneColor(jId, boneC);
        }
    }
}

void SkinMixerManager::colorizeByAnimationWeights(
        ModelDrawer* modelDrawer,
        const std::vector<std::vector<double>>& blendingAnimationWeights)
{
    typedef Model::Skeleton Skeleton;
    typedef Skeleton::JointId JointId;

    if (blendingAnimationWeights.empty()) {
        return;
    }

    const Model& model = *modelDrawer->model();
    const Skeleton& skeleton = model.skeleton;

    std::vector<nvl::Color> blendingColors;

    for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
        //TODO
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

        if (selectedJointId != nvl::MAX_INDEX) {
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
                        if (info.vId == nvl::MAX_INDEX) {
                            if (info.eId == firstEntry) {
                                currentValue += 1 - info.weight;
                            }
                            else {
                                currentValue += info.weight;
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
            loadModel(model);
        }
    }

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetSlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateCanvasPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetResetButton_clicked()
{
    ui->cutOffsetSlider->setValue(0);
    updateCanvasPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutRigiditySlider_valueChanged(int value)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(value);
    updateCanvasPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_cutOffsetRigidityButton_clicked()
{
    ui->cutRigiditySlider->setValue(60);

    updateCanvasPreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationDetachButton_clicked()
{
    vCurrentOperation = OperationType::DETACH;

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationRemoveButton_clicked()
{
    vCurrentOperation = OperationType::REMOVE;

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAttachButton_clicked()
{
    vCurrentOperation = OperationType::ATTACH;

    assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::MAX_INDEX);
    vAttachModelDrawer = vSelectedModelDrawer;
    vAttachJoint = vSelectedJoint;
    vBackupFrame = nvl::Affine3d::Identity();

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAbortButton_clicked()
{
    abortOperation();

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationApplyButton_clicked()
{
    applyOperation();

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::clear()
{
    while (!vModelToDrawerMap.empty()) {
        ModelDrawer* drawable = (vModelToDrawerMap.begin())->second;
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

void SkinMixerManager::clearLayout(QLayout *layout)
{
    QLayoutItem *item;
    while((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
           delete item->widget();
        }
        delete item;
    }
}

void SkinMixerManager::on_mixButton_clicked()
{
    mix();

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_animationJointAllCheckBox_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked) {
        ui->animationJointMeshComboBox->setCurrentIndex(0);

        Model* modelPtr = vSelectedModelDrawer->model();
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);
        std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
            if (jId != vSelectedJoint) {
                for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                    animationWeights[jId][i] = animationWeights[vSelectedJoint][i];
                }
            }
        }

        blendAnimations();
    }
}

void SkinMixerManager::on_animationJointMeshComboBox_currentIndexChanged(int index)
{
    if (index > 0) {
        Index selectedIndex = static_cast<Index>(ui->animationJointMeshComboBox->currentIndex() - 1);

        ui->animationJointAllCheckBox->setChecked(false);

        const Model* modelPtr = vSelectedModelDrawer->model();
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);
        std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

        std::vector<Index> clusterMap = nvl::getInverseMap(entry.birth.entries);
        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
            if (jId != vSelectedJoint) {
                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                for (JointInfo jointInfo : jointInfos) {
                    assert(jointInfo.jId != nvl::MAX_INDEX);
                    assert(jointInfo.eId != nvl::MAX_INDEX);
                    assert(entriesMap[jointInfo.eId] != nvl::MAX_INDEX);

                    const Index& cId = clusterMap[jointInfo.eId];

                    if (cId == selectedIndex && jointInfo.confidence == 1.0) {
                        for (Index i = 0; i < animationWeights[vSelectedJoint].size(); ++i) {
                            animationWeights[jId][i] = animationWeights[vSelectedJoint][i];
                        }

                        break;
                    }
                }
            }
        }

        blendAnimations();
    }
}

void SkinMixerManager::on_animationBlendButton_clicked()
{
    blendAnimations();

    updateCanvasPreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_animationConfirmButton_clicked()
{
    vBlendingAnimation = nvl::MAX_INDEX;

    updateView();
    vCanvas->stopAnimations();
    vCanvas->updateGL();
}

void SkinMixerManager::on_animationAbortButton_clicked()
{
    vSelectedModelDrawer->model()->removeAnimation(vBlendingAnimation);
    vModelAnimationWidget->unselectAnimation();
    vBlendingAnimation = nvl::MAX_INDEX;

    updateView();
    vCanvas->stopAnimations();
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
