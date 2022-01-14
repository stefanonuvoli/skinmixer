#include "skinmixer_manager.h"
#include "ui_skinmixer_manager.h"

#include <nvl/utilities/timer.h>
#include <nvl/utilities/color_utils.h>

#include <nvl/models/io/model_io.h>

#include <QFileDialog>
#include <QMessageBox>

#include <iostream>

#include "skinmixer/skinmixer.h"

#include <nvl/models/algorithms/mesh_normals.h>

#define HARDNESS_DEFAULT 0
#define DETACHING_HARDNESS_DEFAULT 0
#define REMOVING_HARDNESS_DEFAULT 0
#define REPLACING_HARDNESS1_DEFAULT 0
#define REPLACING_HARDNESS2_DEFAULT 0
#define ATTACHING_HARDNESS2_DEFAULT 0

SkinMixerManager::SkinMixerManager(
        nvl::QCanvas* canvas,
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
    vSelectedJoint(nvl::NULL_ID),
    vFirstSelectedModelDrawer(nullptr),
    vFirstSelectedJoint(nvl::NULL_ID),
    vPreparedOperation(false)
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

    nvl::IOModelError error;
    nvl::IOModelMode mode;

    if (ui->modelPoseZeroRadio->isChecked()) {
        mode.FBXDeformToPose = nvl::IOModelFBXPose::IO_FBX_POSE_ZERO;
    }
    else if (ui->modelPoseBindRadio->isChecked()) {
        mode.FBXDeformToPose = nvl::IOModelFBXPose::IO_FBX_POSE_BIND;
    }
    else if (ui->modelPoseDefaultRadio->isChecked()) {
        mode.FBXDeformToPose = nvl::IOModelFBXPose::IO_FBX_POSE_DEFAULT;
    }
    else {
        mode.FBXDeformToPose = nvl::IOModelFBXPose::IO_FBX_POSE_NONE;
    }

    mode.FBXSavePoses = true;

    bool success = nvl::modelLoadFromFile(filename, tmpModel, error, mode);

    if (success) {
        Model* model = new Model(tmpModel);
        initializeLoadedModel(model);

        return loadModel(model);
    }
    else {
        QMessageBox::warning(this, tr("Error"), tr("Error: impossible to load model!"));
        return nvl::NULL_ID;
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

    return vCanvas->addDrawable(modelDrawer, model->name);
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
            if (data[i].identifier == nvl::QCanvas::PICKING_SKELETON_JOINT) {
                selected = i;
                break;
            }
        }

        const PickingData& picked = data[selected];
        size_t drawableId = picked.value(0);

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

        if (picked.identifier == nvl::QCanvas::PICKING_SKELETON_JOINT) {
            Index jointId = picked.value(1);

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

    if (vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID) {
        vSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vSelectedJoint);
    }
    if (vFirstSelectedModelDrawer != nullptr && vFirstSelectedJoint != nvl::NULL_ID) {
        vFirstSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vFirstSelectedJoint);
    }

    vSelectedModelDrawer = getSelectedModelDrawer();
    vSelectedJoint = getSelectedJointId();

    if (vCurrentOperation == OperationType::REPLACE) {
        assert(vFirstSelectedModelDrawer != nullptr);
        prepareModelForReplaceOrAttach();
    }
    else if (vCurrentOperation == OperationType::ATTACH) {
        assert(vFirstSelectedModelDrawer != nullptr);
        prepareModelForReplaceOrAttach();
    }
    else if (vCurrentOperation != OperationType::NONE) {
        abortOperation();
    }

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    updateView();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::slot_animationSelectionChanged(const Index& selectedAnimation)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedAnimation);

    updateView();
}

void SkinMixerManager::slot_drawableSelectionChanged(const std::unordered_set<Index>& selectedDrawables)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(selectedDrawables);

    if (vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID) {
        vSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vSelectedJoint);
    }
    if (vFirstSelectedModelDrawer != nullptr && vFirstSelectedJoint != nvl::NULL_ID) {
        vSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vFirstSelectedJoint);
    }

    vSelectedModelDrawer = getSelectedModelDrawer();
    vSelectedJoint = getSelectedJointId();

    if (vCurrentOperation == OperationType::REPLACE) {
        assert(vFirstSelectedModelDrawer != nullptr);
        prepareModelForReplaceOrAttach();
    }
    else if (vCurrentOperation == OperationType::ATTACH) {
        assert(vFirstSelectedModelDrawer != nullptr);
        prepareModelForReplaceOrAttach();
    }
    else if (vCurrentOperation != OperationType::NONE) {
        abortOperation();
    }

    updateView();
    updatePreview();

    vCanvas->setMovableFrame(nvl::Affine3d::Identity());

    vCanvas->updateGL();
}

void SkinMixerManager::slot_movableFrameChanged()
{
    if (vSelectedModelDrawer == nullptr) {
        return;
    }

    nvl::Affine3d transform = vCanvas->movableFrame();

    nvl::Affine3d::LinearMatrixType scaMatrix;
    transform.computeRotationScaling(static_cast<nvl::Affine3d::LinearMatrixType*>(0), &scaMatrix);
    nvl::Scaling3d sca(scaMatrix.diagonal());
    nvl::Rotation3d rot(transform.rotation());
    nvl::Translation3d tra(transform.translation());

    if (vSelectedJoint != nvl::NULL_ID) {
        Model* modelPtr = vSelectedModelDrawer->model();
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

        if (!tra.isApprox(nvl::Translation3d::Identity()) || !rot.isApprox(nvl::Rotation3d::Identity())) {
            vSkinMixerData.applyJointDeformation(entry, vSelectedJoint, rot, tra);
        }        

        updateAllModelDrawers();

        updatePreview();
    }
    else {
        for (Index id : vDrawableListWidget->selectedDrawables()) {
            if (vCanvas->isFrameable(id)) {
                nvl::Frameable* frameable = vCanvas->frameable(id);
                nvl::Drawable* drawable = vCanvas->drawable(id);

                const nvl::Affine3d& frame = frameable->frame();

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
            modelDrawer->meshDrawer().setFaceTransparency(true);

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
    SkinMixerManager::JointId selectedJointId = nvl::NULL_ID;

    const std::unordered_set<Index>& selectedJoints = vSkeletonJointListWidget->selectedJoints();
    if (selectedJoints.size() == 1) {
        Index firstItem = *selectedJoints.begin();
        selectedJointId = firstItem;
    }

    return selectedJointId;
}

nvl::Index SkinMixerManager::getSelectedAnimation()
{
    return vModelAnimationWidget->selectedAnimation();
}

nvl::Index SkinMixerManager::getCurrentAnimationFrame()
{
    return vModelAnimationWidget->currentAnimationFrame();
}


void SkinMixerManager::mix()
{
    skinmixer::MixParameters par;
    par.mixMode = ui->mixModeMeshingRadioBox->isChecked() ? MixMode::MESHING :  ui->mixModeMorphingRadioBox->isChecked() ? MixMode::MORPHING : MixMode::PREVIEW;
    par.blendColorsFromTextures = ui->blendColorsFromTexturesCheckBox->isChecked();
    par.smoothingBorderIterations = ui->surfaceSmoothingBorderIterationsSpinBox->value();
    par.smoothingBorderThreshold = ui->surfaceSmoothingBorderThresholdSpinBox->value();
    par.smoothingInnerIterations = ui->surfaceSmoothingInnerIterationsSpinBox->value();
    par.smoothingInnerAlpha = ui->surfaceSmoothingInnerAlphaSpinBox->value();
    par.smoothingResultIterations = ui->surfaceSmoothingResultIterationsSpinBox->value();
    par.voxelSize = ui->surfaceVoxelSizeSpinBox->value();
    par.voxelDistance = ui->surfaceVoxelDistanceSpinBox->value();

    std::vector<nvl::Index> newEntries = skinmixer::mix(vSkinMixerData, par);

    for (nvl::Index eId : newEntries) {
        SkinMixerEntry& newEntry = vSkinMixerData.entry(eId);

        Model* newModel = newEntry.model;
        newModel->mesh.computeNormals();

        ModelDrawer* modelDrawer = new ModelDrawer(newModel);
        modelDrawer->meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_VERTEX);
        modelDrawer->meshDrawer().setWireframeVisible(true);

        vDrawerToModelMap.insert(std::make_pair(modelDrawer, newModel));
        vModelToDrawerMap.insert(std::make_pair(newModel, modelDrawer));

        for (const Index& bId : newEntry.birth.entries) {
            Model* model = vSkinMixerData.entry(bId).model;
            typename std::unordered_map<Model*, ModelDrawer*>::iterator it = vModelToDrawerMap.find(model);
            if (it != vModelToDrawerMap.end()) {
                it->second->setVisible(false);
            }
        }

        vCanvas->addDrawable(modelDrawer, "Result");
    }
}

void SkinMixerManager::blendAnimations()
{
    if (!vBlendingAnimations.empty())
        abortAnimations();

    Model* modelPtr = vSelectedModelDrawer->model();
    SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

    skinmixer::MixAnimationParameters parameters;

    parameters.samplingFPS = ui->animationBlendingFPSSpinBox->value();
    parameters.rotationWeight = ui->animationBlendingRotationWeightSpinBox->value();
    parameters.globalWeight = ui->animationBlendingGlobalSpinBox->value();
    parameters.localWeight = ui->animationBlendingLocalSpinBox->value();
    parameters.globalDerivativeWeight = ui->animationBlendingGlobalDerivativeSpinBox->value();
    parameters.localDerivativeWeight = ui->animationBlendingLocalDerivativeSpinBox->value();
    parameters.windowSize = ui->animationBlendingWindowSpinBox->value();
    parameters.windowMainWeight = ui->animationBlendingMainWeightSpinBox->value();
    parameters.smoothingIterations = ui->animationBlendingSmoothIterationsSpinBox->value();
    parameters.smoothingThreshold = ui->animationBlendingSmoothThresholdSpinBox->value();
    skinmixer::mixAnimations(vSkinMixerData, entry, vBlendingAnimations, parameters);

    vCanvas->stopAnimations();

    for (const std::pair<Index, Index>& blendingAnimation : vBlendingAnimations) {
        const Index& entryId = blendingAnimation.first;
        const Index& animationId = blendingAnimation.second;

        const SkinMixerEntry& entry = vSkinMixerData.entry(entryId);
        ModelDrawer* entryModelDrawer = vModelToDrawerMap.at(entry.model);

        entryModelDrawer->loadAnimation(animationId);
    }

    vModelAnimationWidget->updateView();
    vCanvas->updateGL();
}

void SkinMixerManager::abortAnimations()
{
    vCanvas->stopAnimations();
    vModelAnimationWidget->unselectAnimation();

    for (const std::pair<Index, Index>& blendingAnimation : vBlendingAnimations) {
        const Index& entryId = blendingAnimation.first;
        const Index& animationId = blendingAnimation.second;

        const SkinMixerEntry& entry = vSkinMixerData.entry(entryId);
        ModelDrawer* modelDrawer = vModelToDrawerMap.at(entry.model);

        modelDrawer->unloadAnimation();
        entry.model->removeAnimation(animationId);
    }

    vBlendingAnimations.clear();

    vModelAnimationWidget->updateView();
}

//nvl::Index SkinMixerManager::findBestAnimation(const nvl::Index& index)
//{
//    Model* modelPtr = vSelectedModelDrawer->model();
//    SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

//    vBlendingAnimation = vModelAnimationWidget->selectedAnimation();

//    return skinmixer::chooseAnimation(vSkinMixerData, entry, index);
//}

void SkinMixerManager::applyOperation()
{
    if (vCurrentOperation == OperationType::NONE)
        return;

    assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID);

    const unsigned int smoothingIterations = ui->weightSmoothingSlider->value();
    const double rigidity = ui->rigiditySlider->value() / 100.0;

    const double hardness1 = ui->hardness1Slider->value() / 100.0;
    const double includeParent1 = ui->parent1CheckBox->isChecked();

    if (vCurrentOperation == OperationType::REMOVE) {
        skinmixer::remove(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, smoothingIterations, rigidity, hardness1, includeParent1);
    }
    else if (vCurrentOperation == OperationType::DETACH) {
        skinmixer::detach(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, smoothingIterations, rigidity, hardness1, includeParent1);
    }
    else {
        const double hardness2 = ui->hardness2Slider->value() / 100.0;
        const double includeParent2 = ui->parent2CheckBox->isChecked();

        assert(vFirstSelectedModelDrawer != nullptr && vFirstSelectedJoint != nvl::NULL_ID);
        assert(vSelectedModelDrawer != vFirstSelectedModelDrawer);

        if (vCurrentOperation == OperationType::REPLACE) {
            const ReplaceMode replaceMode = ui->replaceModeBlendRadioBox->isChecked() ? ReplaceMode::BLEND : ReplaceMode::UNION;
            skinmixer::replace(vSkinMixerData, vFirstSelectedModelDrawer->model(), vSelectedModelDrawer->model(), vFirstSelectedJoint, vSelectedJoint, replaceMode, smoothingIterations, rigidity, hardness1, hardness2, includeParent1, includeParent2);
        }
        else if (vCurrentOperation == OperationType::ATTACH) {
            skinmixer::attach(vSkinMixerData, vFirstSelectedModelDrawer->model(), vSelectedModelDrawer->model(), vFirstSelectedJoint, vSelectedJoint, smoothingIterations, rigidity, hardness2, includeParent1, includeParent2);
        }

        vFirstSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());
        updateModelDrawer(vFirstSelectedModelDrawer);

        updateModelDrawer(vFirstSelectedModelDrawer);
        if (vFirstSelectedJoint != nvl::NULL_ID) {
            vFirstSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vFirstSelectedJoint);
        }

        vFirstSelectedModelDrawer = nullptr;
        vFirstSelectedJoint = nvl::NULL_ID;
        vPreparedOperation = false;
    }

    updateModelDrawer(vSelectedModelDrawer);
    vSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());
    vCurrentOperation = OperationType::NONE;
}

void SkinMixerManager::abortOperation()
{
    vCurrentOperation = OperationType::NONE;
    ui->hardness1Slider->setValue(HARDNESS_DEFAULT);
    ui->hardness2Slider->setValue(HARDNESS_DEFAULT);

    if (vSelectedModelDrawer != nullptr) {
        updateModelDrawer(vSelectedModelDrawer);
    }
    if (vFirstSelectedModelDrawer != nullptr) {
        updateModelDrawer(vFirstSelectedModelDrawer);
        if (vFirstSelectedJoint != nvl::NULL_ID) {
            vFirstSelectedModelDrawer->skeletonDrawer().resetRenderingJointColor(vFirstSelectedJoint);
        }
    }

    vFirstSelectedModelDrawer = nullptr;
    vFirstSelectedJoint = nvl::NULL_ID;
    vPreparedOperation = false;
}

void SkinMixerManager::updatePreview()
{
    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::NULL_ID;

    bool firstDrawableSelected = vFirstSelectedModelDrawer != nullptr;
    bool firstJointSelected = firstDrawableSelected && vFirstSelectedJoint != nvl::NULL_ID;

    if (vCurrentOperation != OperationType::NONE && jointSelected) {        
        nvl::Index actionId = nvl::NULL_ID;

        const unsigned int smoothingIterations = ui->weightSmoothingSlider->value();
        const double rigidity = ui->rigiditySlider->value() / 100.0;

        const double hardness1 = ui->hardness1Slider->value() / 100.0;
        const double includeParent1 = ui->parent1CheckBox->isChecked();

        if (vCurrentOperation == OperationType::REMOVE) {
            actionId = skinmixer::remove(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, smoothingIterations, rigidity, hardness1, includeParent1);
        }
        else if (vCurrentOperation == OperationType::DETACH) {
            actionId = skinmixer::detach(vSkinMixerData, vSelectedModelDrawer->model(), vSelectedJoint, smoothingIterations, rigidity, hardness1, includeParent1);
        }
        else if ((vCurrentOperation == OperationType::REPLACE || vCurrentOperation == OperationType::ATTACH) && firstDrawableSelected && firstJointSelected && vSelectedModelDrawer != vFirstSelectedModelDrawer) {
            const double hardness2 = ui->hardness2Slider->value() / 100.0;
            const double includeParent2 = ui->parent2CheckBox->isChecked();

            if (vCurrentOperation == OperationType::REPLACE) {
                const ReplaceMode replaceMode = ui->replaceModeBlendRadioBox->isChecked() ? ReplaceMode::BLEND : ReplaceMode::UNION;
                actionId = skinmixer::replace(vSkinMixerData, vFirstSelectedModelDrawer->model(), vSelectedModelDrawer->model(), vFirstSelectedJoint, vSelectedJoint, replaceMode, smoothingIterations, rigidity, hardness1, hardness2, includeParent1, includeParent2);
            }
            else if (vCurrentOperation == OperationType::ATTACH) {
                actionId = skinmixer::attach(vSkinMixerData, vFirstSelectedModelDrawer->model(), vSelectedModelDrawer->model(), vFirstSelectedJoint, vSelectedJoint, smoothingIterations, rigidity, hardness2, includeParent1, includeParent2);
            }

            if (actionId != nvl::NULL_ID) {
                updateModelDrawer(vFirstSelectedModelDrawer);
            }
        }

        if (actionId != nvl::NULL_ID) {
            updateModelDrawer(vSelectedModelDrawer);
            vSkinMixerData.removeAction(actionId);
        }
    }

    if (jointSelected) {
        vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vSelectedJoint, nvl::Color(1.0, 1.0, 0.0));
    }
    if (firstJointSelected) {
        vFirstSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vFirstSelectedJoint, nvl::Color(0.0, 1.0, 1.0));
    }
}

void SkinMixerManager::prepareModelForReplaceOrAttach()
{
    bool modelDrawerSelected = vSelectedModelDrawer != nullptr;
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::NULL_ID;

    if (jointSelected) {
        if (!vPreparedOperation && vSelectedModelDrawer != vFirstSelectedModelDrawer) {
            typedef typename Model::Skeleton Skeleton;
            typedef typename Skeleton::JointId JointId;

            const Model* model1 = vFirstSelectedModelDrawer->model();
            const Skeleton& skeleton1 = model1->skeleton;
            JointId targetJoint1 = vFirstSelectedJoint;

            const Model* model2 = vSelectedModelDrawer->model();
            const Skeleton& skeleton2 = model2->skeleton;
            JointId targetJoint2 = vSelectedJoint;

            const double includeParent1 = ui->parent1CheckBox->isChecked();
            const double includeParent2 = ui->parent2CheckBox->isChecked();
            if (includeParent1) {
                targetJoint1 = skeleton1.parentId(targetJoint1);
            }
            if (includeParent2) {
                targetJoint2 = skeleton2.parentId(targetJoint2);
            }

            vPreparedOperation = true;

            vSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());
            vFirstSelectedModelDrawer->setFrame(nvl::Affine3d::Identity());
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
    bool jointSelected = modelDrawerSelected && vSelectedJoint != nvl::NULL_ID;
    bool blendedModelSelected = modelDrawerSelected && !vSkinMixerData.entryFromModel(vSelectedModelDrawer->model()).birth.entries.empty();
    bool animationBlending = blendedModelSelected && !vBlendingAnimations.empty();
    bool animationSelected = getSelectedAnimation() != nvl::NULL_ID;

    ui->modelLoadButton->setEnabled(true);
    ui->modelRemoveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelSaveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelMoveButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelCopyButton->setEnabled(atLeastOneDrawableSelected);
    ui->scaleOn1AndCenterButton->setEnabled(atLeastOneDrawableSelected);
    ui->modelAnimationPoseButton->setEnabled(animationSelected);
    ui->modelAnimationRemoveButton->setEnabled(animationSelected);
    ui->modelAnimationRemoveRootMotionButton->setEnabled(animationSelected);

    ui->operationDetachButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationRemoveButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationReplaceButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationAttachButton->setEnabled(jointSelected && vCurrentOperation == OperationType::NONE);
    ui->operationAbortButton->setEnabled(jointSelected && vCurrentOperation != OperationType::NONE);
    ui->operationApplyButton->setEnabled(jointSelected && (
        (vCurrentOperation == OperationType::DETACH || vCurrentOperation == OperationType::REMOVE) ||
        ((vCurrentOperation == OperationType::REPLACE || vCurrentOperation == OperationType::ATTACH) && vFirstSelectedModelDrawer != vSelectedModelDrawer)
    ));
    ui->resetJointDeformationsButton->setEnabled(atLeastOneDrawableSelected && vCurrentOperation == OperationType::NONE);
    ui->mixButton->setEnabled(vCurrentOperation == OperationType::NONE && !vSkinMixerData.actions().empty());

    ui->hardness1Slider->setEnabled(jointSelected && vCurrentOperation != OperationType::NONE && vCurrentOperation != OperationType::ATTACH);
    ui->hardness2Slider->setEnabled(jointSelected && (vCurrentOperation == OperationType::REPLACE || vCurrentOperation == OperationType::ATTACH));

    ui->updateValuesWeightsButton->setEnabled(jointSelected);
    ui->updateValuesBirthButton->setEnabled(modelDrawerSelected);

    ui->animationBlendingFrame->setEnabled(blendedModelSelected);
    ui->animationSelectGroupBox->setEnabled(blendedModelSelected && !animationBlending);
    ui->animationBlendButton->setEnabled(blendedModelSelected && !animationBlending);
    ui->animationBlendLoopFixedButton->setEnabled(blendedModelSelected && !animationBlending);
    ui->animationConfirmButton->setEnabled(animationBlending);
    ui->animationAbortButton->setEnabled(animationBlending);

    ui->animationJointFrame->setEnabled(blendedModelSelected && jointSelected);
    ui->animationJointGroupBox->setEnabled(blendedModelSelected && jointSelected);

    ui->updateJointsBirthButton->setEnabled(blendedModelSelected && jointSelected);

    clearLayout(ui->animationSelectGroupBox->layout());
    clearLayout(ui->animationJointGroupBox->layout());
    ui->animationJointMeshComboBox->clear();

    on_rigiditySlider_valueChanged(ui->rigiditySlider->value());
    on_hardness1Slider_valueChanged(ui->hardness1Slider->value());
    on_hardness2Slider_valueChanged(ui->hardness2Slider->value());
    on_weightSmoothingSlider_valueChanged(ui->weightSmoothingSlider->value());

    ui->actionTreeWidget->clear();
    for (const SkinMixerAction& action : vSkinMixerData.actions()) {
        std::string type;
        if (action.operation == OperationType::REPLACE) {
            type = "Replace";
        }
        else if (action.operation == OperationType::ATTACH) {
            type = "Attach";
        }
        else if (action.operation == OperationType::REMOVE) {
            type = "Remove";
        }
        else if (action.operation == OperationType::DETACH) {
            type = "Detach";
        }
        else {
            type = "Unknown";
        }

        QTreeWidgetItem* item = new QTreeWidgetItem();
        item->setText(0, QString(type.c_str()));
        item->setText(1, QString(std::to_string(action.entry1).c_str()));
        item->setText(2, QString(std::to_string(action.joint1).c_str()));
        item->setText(3, QString(std::to_string(action.entry2).c_str()));
        item->setText(4, QString(std::to_string(action.joint2).c_str()));
        ui->actionTreeWidget->addTopLevelItem(item);
    }

    if (blendedModelSelected) {
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(vSelectedModelDrawer->model());
        std::vector<double>& animationSpeeds = entry.blendingAnimationSpeeds;

        const std::vector<Index>& birthEntries = entry.birth.entries;

        std::vector<Index>& animationIds = entry.blendingAnimationIds;
        std::vector<Index>& animationModes = entry.blendingAnimationModes;

        std::vector<QDoubleSpinBox*> animationSpeedSpinBoxes(birthEntries.size());
        for (Index cId = 0; cId < birthEntries.size(); ++cId) {
            const Index& eId = birthEntries[cId];
            Model* currentModel = vSkinMixerData.entry(eId).model;

            QLabel* animationNameLabel = new QLabel(currentModel->name.c_str());

            QComboBox* animationModeCombo = new QComboBox(this);
            animationModeCombo->addItem("Fixed pose");
            animationModeCombo->addItem("Best keyframes");
            animationModeCombo->addItem("Best loop");

            QComboBox* animationIdCombo = new QComboBox(this);
            animationIdCombo->addItem("None");
            for (Index aId = 0; aId < currentModel->animationNumber(); aId++) {
                const Animation& animation = currentModel->animation(aId);
                animationIdCombo->addItem(animation.name().c_str());
            }

            animationModeCombo->setCurrentIndex(animationModes[cId]);
            if (animationIds[cId] == BLEND_ANIMATION_NONE) {
                animationIdCombo->setCurrentIndex(0);
            }
            else {
                animationIdCombo->setCurrentIndex(animationIds[cId] + 1);
            }

            QDoubleSpinBox* animationSpeedSpinBox = new QDoubleSpinBox(this);
            animationSpeedSpinBox->setMinimum(0.0);
            animationSpeedSpinBox->setSingleStep(0.05);
            animationSpeedSpinBox->setMaximum(100.0);
            animationSpeedSpinBox->setDecimals(5);
            animationSpeedSpinBox->setValue(animationSpeeds[cId]);
            animationSpeedSpinBoxes[cId] = animationSpeedSpinBox;


            QComboBox::connect(animationModeCombo, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            [&animationModes, cId, this](int index) {
                animationModes[cId] = index;

                if (!vBlendingAnimations.empty()) {
                    blendAnimations();
                }
            });

            QComboBox::connect(animationIdCombo, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            [&animationIds, cId, this](int index) {
                if (index == 0) {
                    animationIds[cId] = BLEND_ANIMATION_NONE;
                }
                else {
                    animationIds[cId] = index - 1;
                }

                if (!vBlendingAnimations.empty()) {
                    blendAnimations();
                }
            });

            void (QDoubleSpinBox:: *signal)(double) = static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged);
            QSlider::connect(animationSpeedSpinBox, signal,
            [&animationSpeeds, cId, entry, this](double value) {
                animationSpeeds[cId] = value;

                if (!vBlendingAnimations.empty()) {
                    blendAnimations();
                }
            });


            QFrame* frame = new QFrame();
            QVBoxLayout *layout = new QVBoxLayout;
            frame->setLayout(layout);
            frame->layout()->addWidget(animationNameLabel);
            frame->layout()->addWidget(animationModeCombo);
            frame->layout()->addWidget(animationIdCombo);
            frame->layout()->addWidget(animationSpeedSpinBox);

            ui->animationSelectGroupBox->layout()->addWidget(frame);
        }


        if (jointSelected) {
            std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

            animationWeightSliders.resize(birthEntries.size());
            animationWeightLabels.resize(birthEntries.size());
            for (Index cId = 0; cId < birthEntries.size(); ++cId) {
                const Index& eId = birthEntries[cId];
                Model* currentModel = vSkinMixerData.entry(eId).model;

                QLabel* label = new QLabel(currentModel->name.c_str());
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
                        std::vector<Index> clusterMap = nvl::inverseFunction(entry.birth.entries);
                        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
                            if (jId != vSelectedJoint) {
                                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                                for (JointInfo jointInfo : jointInfos) {
                                    assert(jointInfo.jId != nvl::NULL_ID);
                                    assert(jointInfo.eId != nvl::NULL_ID);
                                    assert(clusterMap[jointInfo.eId] != nvl::NULL_ID);

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

                    if (!vBlendingAnimations.empty()) {
                        blendAnimations();
                    }
                });
            }

            ui->animationJointMeshComboBox->addItem("None");
            for (const Index& eId : birthEntries) {
                Model* currentModel = vSkinMixerData.entry(eId).model;
                ui->animationJointMeshComboBox->addItem(currentModel->name.c_str());
            }
        }
    }
}

void SkinMixerManager::updateAllModelDrawers()
{
    for (const SkinMixerEntry& entry : vSkinMixerData.entries()) {
        std::unordered_map<Model*, ModelDrawer*>::iterator it = vModelToDrawerMap.find(entry.model);
        if (it != vModelToDrawerMap.end()) {
            updateModelDrawer(it->second);
        }
    }
}

void SkinMixerManager::updateModelDrawer(
        ModelDrawer* modelDrawer)
{
    SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelDrawer->model());

    //Reset of the model drawer
    modelDrawer->update();

    SelectInfo selectInfo = vSkinMixerData.computeGlobalSelectInfo(entry);

    colorizeBySelectValues(
        modelDrawer,
        selectInfo.vertex,
        selectInfo.joint);

    colorizeByAnimationWeights(
        modelDrawer,
        entry.blendingAnimationWeights);

    std::vector<nvl::DualQuaterniond> deformations = vSkinMixerData.computeDeformation(entry);
    if (!deformations.empty()) {
        modelDrawer->renderDualQuaternionSkinning(deformations);
    }

    if (vSelectedModelDrawer == modelDrawer && vSelectedJoint != nvl::NULL_ID) {
        vSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vSelectedJoint, nvl::Color(1.0, 1.0, 0.0));
    }
    if (vFirstSelectedModelDrawer == modelDrawer && vFirstSelectedJoint != nvl::NULL_ID) {
        vFirstSelectedModelDrawer->skeletonDrawer().setRenderingJointColor(vFirstSelectedJoint, nvl::Color(0.0, 1.0, 1.0));
    }
}

void SkinMixerManager::colorizeBySelectValues(
        ModelDrawer* modelDrawer,
        const std::vector<double>& vertexSelectValue,
        const std::vector<double>& jointSelectValue)
{
    typedef Model::Mesh Mesh;
    typedef Model::Skeleton Skeleton;
    typedef Mesh::VertexId VertexId;
    typedef Mesh::FaceId FaceId;
    typedef Mesh::Face Face;
    typedef Skeleton::JointId JointId;

    assert(!vertexSelectValue.empty() && !jointSelectValue.empty());

    double minAlpha = 0.0;
    if (ui->showZeroCheckBox->isChecked()) {
        minAlpha = 0.2;
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
        wireframeC.setAlphaF(std::max(minAlpha, avgValue));
        modelDrawer->meshDrawer().setRenderingFaceWireframeColor(fId, wireframeC);
    }

    if (!mesh.hasVertexColors()) {
        modelDrawer->meshDrawer().setRenderingVertexColors(std::vector<float>(mesh.vertexNumber() * 4));
    }
    for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
        if (mesh.isVertexDeleted(vId))
            continue;

        double value = std::max(std::min(vertexSelectValue[vId], 1.0), 0.0);

        nvl::Color vertexC;
        if (mesh.hasVertexColors()) {
            vertexC = modelDrawer->meshDrawer().renderingVertexColor(vId);
        }
        else {
            vertexC = nvl::Color(1.0, 1.0, 1.0);
        }

        vertexC.setAlphaF(std::max(minAlpha, value));
        modelDrawer->meshDrawer().setRenderingVertexColor(vId, vertexC);
    }


    for (JointId jId = 0; jId < skeleton.jointNumber(); jId++) {
        double jointValue = std::max(std::min(jointSelectValue[jId], 1.0), 0.0);

        nvl::Color jointC = modelDrawer->skeletonDrawer().renderingJointColor(jId);
        jointC.setAlphaF(std::max(minAlpha, jointValue));
        modelDrawer->skeletonDrawer().setRenderingJointColor(jId, jointC);

        if (!skeleton.isRoot(jId)) {
            JointId parentJointId = skeleton.parentId(jId);
            double boneValue = std::max(std::min(jointValue, jointSelectValue[parentJointId]), 0.0);

            nvl::Color boneC = modelDrawer->skeletonDrawer().renderingBoneColor(jId);
            boneC.setAlphaF(std::max(minAlpha, boneValue));
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
    if (vSelectedModelDrawer != nullptr) {
        vSelectedModelDrawer->meshDrawer().clearVertexValues();

        vCanvas->updateGL();
    }
}

void SkinMixerManager::updateValuesSkinningWeights()
{
    if (vSelectedModelDrawer != nullptr) {
        std::vector<double> vertexValues;

        if (vSelectedJoint != nvl::NULL_ID) {
            vertexValues.resize(vSelectedModelDrawer->model()->mesh.nextVertexId(), 0.0);

            for (auto vertex : vSelectedModelDrawer->model()->mesh.vertices()) {
                vertexValues[vertex.id()] = vSelectedModelDrawer->model()->skinningWeights.weight(vertex.id(), vSelectedJoint);
            }
        }

        vSelectedModelDrawer->meshDrawer().setVertexValues(vertexValues);

        vCanvas->updateGL();
    }
}

void SkinMixerManager::updateValuesSelect()
{
    if (vSelectedModelDrawer != nullptr) {
        Model* modelPtr = vSelectedModelDrawer->model();
        const SkinMixerEntry& currentEntry = vSkinMixerData.entryFromModel(modelPtr);
        SelectInfo currentSelect = vSkinMixerData.computeGlobalSelectInfo(currentEntry);

        std::vector<double> vertexValues;

        vertexValues.resize(vSelectedModelDrawer->model()->mesh.nextVertexId(), 1.0);

        for (auto vertex : vSelectedModelDrawer->model()->mesh.vertices()) {
            vertexValues[vertex.id()] = currentSelect.vertex[vertex.id()];
        }

        vSelectedModelDrawer->meshDrawer().setVertexValues(vertexValues);

        vCanvas->updateGL();
    }
}

void SkinMixerManager::updateValuesBirth()
{
    if (vSelectedModelDrawer != nullptr) {
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(vSelectedModelDrawer->model());

        if (!entry.birth.vertex.empty()) {
            std::set<nvl::Index> birthEntries;
            for (auto vertex : vSelectedModelDrawer->model()->mesh.vertices()) {
                for (auto info : entry.birth.vertex[vertex.id()]) {
                    birthEntries.insert(info.eId);
                }
            }

            if (birthEntries.size() > 2) {
                vSelectedModelDrawer->meshDrawer().clearVertexValues();
            }
            else {
                nvl::Index firstEntry = *birthEntries.begin();

                std::vector<double> vertexValues;
                vertexValues.resize(vSelectedModelDrawer->model()->mesh.nextVertexId(), -1.0);

                for (auto vertex : vSelectedModelDrawer->model()->mesh.vertices()) {
                    double currentValue = 0.0;

                    nvl::Size n = 0;

                    for (auto info : entry.birth.vertex[vertex.id()]) {
                        if (info.eId == firstEntry) {
                            currentValue += 1 - info.weight;
                        }
                        else {
                            currentValue += info.weight;
                        }
                        n++;
                    }

                    currentValue /= n;

                    vertexValues[vertex.id()] = currentValue;
                }

                vSelectedModelDrawer->meshDrawer().setVertexValues(vertexValues);
            }

            vCanvas->updateGL();
        }
    }
}

void SkinMixerManager::updateJointsReset()
{
    updateAllModelDrawers();

    vCanvas->updateGL();
}

void SkinMixerManager::updateJointsBirth()
{
    updateAllModelDrawers();

    if (vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID) {
        SkinMixerEntry& entry = vSkinMixerData.entryFromModel(vSelectedModelDrawer->model());

        if (!entry.birth.joint.empty()) {
            for (auto jointInfo : entry.birth.joint[vSelectedJoint]) {
                const SkinMixerEntry& birthEntry = vSkinMixerData.entry(jointInfo.eId);
                ModelDrawer* modelDrawer = vModelToDrawerMap.at(birthEntry.model);

                nvl::Color color = nvl::getRampRedGreen(jointInfo.confidence);
                modelDrawer->skeletonDrawer().setRenderingJointColor(jointInfo.jId, color);
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
        connect(vModelAnimationWidget, &nvl::ModelAnimationWidget::signal_animationSelectionChanged, this, &SkinMixerManager::slot_animationSelectionChanged);
        connect(this, &SkinMixerManager::signal_selectedDrawableUpdated, vModelAnimationWidget, &nvl::ModelAnimationWidget::slot_selectedDrawableUpdated);
        connect(vCanvas, &nvl::QCanvas::signal_movableFrameChanged, this, &SkinMixerManager::slot_movableFrameChanged);
        connect(vCanvas, &nvl::QCanvas::signal_canvasPicking, this, &SkinMixerManager::slot_canvasPicking);
        connect(vCanvas, &nvl::QCanvas::signal_drawableAdded, this, &SkinMixerManager::slot_drawableAdded);
    }
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
    if (ui->updateFaceNormalsCheckBox->isChecked()) {
        model->mesh.computeFaceNormals();
    }
    if (ui->updateVertexNormalsCheckBox->isChecked()) {
        model->mesh.computeVertexNormals();
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

void SkinMixerManager::on_modelLoadButton_clicked()
{
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    QStringList filters;
    filters
            << "Model (*.rig *.fbx *.RIG *.FBX)"
            << "RIG (*.rig *.RIG)"
            << "FBX (*.fbx *.FBX)"
            << "Any file (*)";
    dialog.setNameFilters(filters);

    QStringList files;

    if (dialog.exec()) {
        files = dialog.selectedFiles();
    }

    for (const QString& str : files) {
        loadModelFromFile(str.toStdString());
    }

    vCanvas->updateGL();
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
    if (vSelectedModelDrawer == nullptr)
        return;

    const Model& model = *vSelectedModelDrawer->model();

    QString filename = QFileDialog::getSaveFileName(this,
        tr("Save model"), QDir::homePath() + "/" + QString(model.name.c_str()) + QString(".rig"),
        tr("Model (*.rig *.RIG *.obj *.OBJ *.fbx *.FBX);;RIG (*.rig *.RIG);;FBX (*.fbx *.FBX);;OBJ (*.obj *.OBJ);;Any file (*)")
    );

    if (!filename.isEmpty()) {
        bool success = nvl::modelSaveToFile(filename.toStdString(), model);
        if (!success) {
            QMessageBox::warning(this, tr("Error"), tr("Error: impossible to save model!"));
        }
    }
}

void SkinMixerManager::on_modelMoveButton_clicked()
{
    const std::unordered_set<Index> selectedDrawables = vDrawableListWidget->selectedDrawables();
    for (Index selected : selectedDrawables) {
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(vCanvas->drawable(selected));
        if (modelDrawer != nullptr) {
            Model* modelPtr = modelDrawer->model();

            nvl::modelApplyTransformation(*modelPtr, modelDrawer->frame());

            modelPtr->mesh.computeFaceNormals();
            modelPtr->mesh.computeVertexNormals();

            modelDrawer->setFrame(nvl::Affine3d::Identity());
        }
    }

    updateAllModelDrawers();

    vCanvas->updateGL();
}

void SkinMixerManager::on_modelCopyButton_clicked()
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

void SkinMixerManager::on_scaleOn1AndCenterButton_clicked()
{
    const std::unordered_set<Index> selectedDrawables = vDrawableListWidget->selectedDrawables();
    for (Index selected : selectedDrawables) {
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(vCanvas->drawable(selected));
        if (modelDrawer != nullptr) {
            Model* modelPtr = modelDrawer->model();

            nvl::AlignedBox3d bbox = nvl::meshBoundingBox(modelPtr->mesh);
            double scaleFactor = 1.0 / bbox.diagonal().norm();
            nvl::Point3d translateVector = -bbox.center();

            nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);
            nvl::Translation3d translateTransform(translateVector);

            std::cout << modelPtr->name << " (V: " << modelPtr->mesh.vertexNumber() << ", F: " << modelPtr->mesh.faceNumber() << ") -> Scale: " << scaleFactor << ", Translation: " << translateTransform.translation().transpose() << std::endl;

            nvl::Affine3d transformation(scaleTransform * translateTransform);
            nvl::modelApplyTransformation(*modelPtr, transformation);

            modelDrawer->update();
        }
    }

    vCanvas->updateGL();
}

void SkinMixerManager::on_weightSmoothingSlider_valueChanged(int value)
{
    std::ostringstream out;
    out.precision(2);
    out << std::fixed << value / 100.0;

    ui->weightSmoothingValueLabel->setText(out.str().c_str());

    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_rigiditySlider_valueChanged(int value)
{
    std::ostringstream out;
    out.precision(2);
    out << std::fixed << value / 100.0;

    ui->rigidityValueLabel->setText(out.str().c_str());

    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_showZeroCheckBox_clicked()
{
    updateAllModelDrawers();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_hardness1Slider_valueChanged(int value)
{
    std::ostringstream out;
    out.precision(2);
    out << (value >= 0 ? "+" : "") << std::fixed << value / 100.0;

    ui->hardness1ValueLabel->setText(out.str().c_str());
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_hardness2Slider_valueChanged(int value)
{
    std::ostringstream out;
    out.precision(2);
    out << (value >= 0 ? "+" : "") << std::fixed << value / 100.0;

    ui->hardness2ValueLabel->setText(out.str().c_str());
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_parent1CheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);

    abortOperation();

    updateView();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_parent2CheckBox_stateChanged(int arg1)
{
    NVL_SUPPRESS_UNUSEDVARIABLE(arg1);

    abortOperation();

    updateView();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationDetachButton_clicked()
{
    vCurrentOperation = OperationType::DETACH;
    ui->hardness1Slider->setValue(DETACHING_HARDNESS_DEFAULT);

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationRemoveButton_clicked()
{
    vCurrentOperation = OperationType::REMOVE;
    ui->hardness1Slider->setValue(REMOVING_HARDNESS_DEFAULT);

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationReplaceButton_clicked()
{
    vCurrentOperation = OperationType::REPLACE;
    ui->hardness1Slider->setValue(REPLACING_HARDNESS1_DEFAULT);
    ui->hardness2Slider->setValue(REPLACING_HARDNESS2_DEFAULT);

    assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID);
    vFirstSelectedModelDrawer = vSelectedModelDrawer;
    vFirstSelectedJoint = vSelectedJoint;

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAttachButton_clicked()
{
    vCurrentOperation = OperationType::ATTACH;
    ui->hardness2Slider->setValue(ATTACHING_HARDNESS2_DEFAULT);

    assert(vSelectedModelDrawer != nullptr && vSelectedJoint != nvl::NULL_ID);
    vFirstSelectedModelDrawer = vSelectedModelDrawer;
    vFirstSelectedJoint = vSelectedJoint;

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationAbortButton_clicked()
{
    abortOperation();

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_operationApplyButton_clicked()
{
    applyOperation();

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_resetJointDeformationsButton_clicked()
{
    const std::unordered_set<Index> selectedDrawables = vDrawableListWidget->selectedDrawables();
    for (Index selected : selectedDrawables) {
        ModelDrawer* modelDrawer = dynamic_cast<ModelDrawer*>(vCanvas->drawable(selected));
        if (modelDrawer != nullptr) {
            Model* modelPtr = modelDrawer->model();
            SkinMixerEntry& entry = vSkinMixerData.entryFromModel(modelPtr);

            vSkinMixerData.resetJointDeformation(entry);

            updateModelDrawer(modelDrawer);
        }
    }

    vCanvas->updateGL();
}

void SkinMixerManager::on_mixButton_clicked()
{
    mix();

    updatePreview();
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

        if (!vBlendingAnimations.empty()) {
            blendAnimations();
        }
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

        std::vector<Index> clusterMap = nvl::inverseFunction(entry.birth.entries);
        for (JointId jId = 0; jId < vSelectedModelDrawer->model()->skeleton.jointNumber(); jId++) {
            if (jId != vSelectedJoint) {
                const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

                for (JointInfo jointInfo : jointInfos) {
                    assert(jointInfo.jId != nvl::NULL_ID);
                    assert(jointInfo.eId != nvl::NULL_ID);
                    assert(clusterMap[jointInfo.eId] != nvl::NULL_ID);

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

        if (!vBlendingAnimations.empty()) {
            blendAnimations();
        }
    }
}

void SkinMixerManager::on_animationBlendButton_clicked()
{
    blendAnimations();

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_animationConfirmButton_clicked()
{
    vCanvas->stopAnimations();

    vBlendingAnimations.clear();

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_animationAbortButton_clicked()
{
    abortAnimations();

    updatePreview();
    updateView();

    vCanvas->updateGL();
}

void SkinMixerManager::on_animationBlendLoopFixedButton_clicked()
{
    ModelDrawer* vSelectedModelDrawer = getSelectedModelDrawer();
    if (vSelectedModelDrawer == nullptr)
        return;

    SkinMixerEntry& entry = vSkinMixerData.entryFromModel(vSelectedModelDrawer->model());

    const std::vector<Index>& birthEntries = entry.birth.entries;
    std::vector<Index>& animationIds = entry.blendingAnimationIds;
    const std::vector<Index>& animationModes = entry.blendingAnimationModes;

    for (Index i = 0; i < birthEntries.size(); ++i) {
        if (animationModes[i] == BLEND_ANIMATION_FIXED && animationIds[i] == BLEND_ANIMATION_NONE) {
            for (Index aId = 0; aId < vSkinMixerData.entry(birthEntries[i]).lastOriginalAnimationId; ++aId) {
                animationIds[i] = aId;
                blendAnimations();

                vSelectedModelDrawer->unloadAnimation();
                vCanvas->stopAnimations();

                vBlendingAnimations.clear();
            }

            animationIds[i] = BLEND_ANIMATION_NONE;
        }
    }

    updatePreview();
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

void SkinMixerManager::on_updateValuesSelectButton_clicked()
{
    updateValuesSelect();
}

void SkinMixerManager::on_updateValuesBirthButton_clicked()
{
    updateValuesBirth();
}

void SkinMixerManager::on_updateJointsResetButton_clicked()
{
    updateJointsReset();
}

void SkinMixerManager::on_updateJointsBirthButton_clicked()
{
    updateJointsBirth();
}

void SkinMixerManager::on_modelAnimationPoseButton_clicked()
{
    typedef typename Model::Animation Animation;
    typedef typename Animation::Transformation Transformation;

    ModelDrawer* vSelectedModelDrawer = getSelectedModelDrawer();
    if (vSelectedModelDrawer == nullptr)
        return;

    Model* model = vSelectedModelDrawer->model();
    if (model == nullptr)
        return;

    Index aId = getSelectedAnimation();
    if (aId == nvl::NULL_ID)
        return;

    Index fId = getCurrentAnimationFrame();
    if (fId == nvl::NULL_ID)
        return;

    std::vector<Transformation> transformations = vSelectedModelDrawer->currentFrame().transformations();
    nvl::modelDeformDualQuaternionSkinning(*model, transformations, true, true);

    vSelectedModelDrawer->update();
    vCanvas->updateGL();
}


void SkinMixerManager::on_modelAnimationRemoveButton_clicked()
{
     ModelDrawer* vSelectedModelDrawer = getSelectedModelDrawer();
     if (vSelectedModelDrawer == nullptr)
         return;

     Model* model = vSelectedModelDrawer->model();
     if (model == nullptr)
         return;

     Index aId = getSelectedAnimation();
     if (aId == nvl::NULL_ID)
         return;

     vSelectedModelDrawer->unloadAnimation();
     model->removeAnimation(aId);

     vCanvas->updateGL();

     emit signal_selectedDrawableUpdated();
}

void SkinMixerManager::on_modelAnimationRemoveRootMotionButton_clicked()
{
    ModelDrawer* vSelectedModelDrawer = getSelectedModelDrawer();
    if (vSelectedModelDrawer == nullptr)
        return;

    Model* model = vSelectedModelDrawer->model();
    if (model == nullptr)
        return;

    Index aId = getSelectedAnimation();
    if (aId == nvl::NULL_ID)
        return;

    nvl::animationRemoveRootMotion(model->skeleton, model->animation(aId));
    vSelectedModelDrawer->reloadAnimation();
}


void SkinMixerManager::on_actionRemoveButton_clicked()
{
    ui->actionTreeWidget->selectedItems();
    QList<QTreeWidgetItem*> items = ui->actionTreeWidget->selectedItems();

    for (int i = 0; i < items.size(); ++i) {
        QTreeWidgetItem* item = items[i];
        int id = ui->actionTreeWidget->indexOfTopLevelItem(item);

        if (id >= 0) {
            vSkinMixerData.removeAction(id);
        }
    }

    updateAllModelDrawers();
    updateView();
    updatePreview();

    vCanvas->updateGL();
}


void SkinMixerManager::on_actionDownButton_clicked()
{
    ui->actionTreeWidget->selectedItems();
    QList<QTreeWidgetItem*> items = ui->actionTreeWidget->selectedItems();

    for (int i = 0; i < items.size(); ++i) {
        QTreeWidgetItem* item = items[i];
        int id = ui->actionTreeWidget->indexOfTopLevelItem(item);

        if (id >= 0 && id + 1 < static_cast<int>(vSkinMixerData.actionNumber())) {
            std::swap(vSkinMixerData.action(id), vSkinMixerData.action(id + 1));
        }
    }

    updateAllModelDrawers();
    updateView();
    updatePreview();

    vCanvas->updateGL();
}

void SkinMixerManager::on_actionUpButton_clicked()
{
    ui->actionTreeWidget->selectedItems();
    QList<QTreeWidgetItem*> items = ui->actionTreeWidget->selectedItems();

    for (int i = 0; i < items.size(); ++i) {
        QTreeWidgetItem* item = items[i];
        int id = ui->actionTreeWidget->indexOfTopLevelItem(item);

        if (id >= 1) {
            std::swap(vSkinMixerData.action(id), vSkinMixerData.action(id - 1));
        }
    }

    updateAllModelDrawers();
    updateView();
    updatePreview();

    vCanvas->updateGL();
}

