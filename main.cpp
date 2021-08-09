#include <QApplication>

#include <nvl/viewer/viewerwindow.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/drawable_widget.h>
#include <nvl/viewer/widgets/skeletonjoint_list_widget.h>
#include <nvl/viewer/widgets/animation_widget.h>
#include <nvl/viewer/widgets/model_animation_widget.h>
#include <nvl/viewer/widgets/model_drawer_widget.h>
#include <nvl/viewer/widgets/qglviewer_canvas.h>

#include "widgets/skinmixer_manager.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    //Viewer
    nvl::ViewerWindow viewer;
    viewer.setWindowTitle("SkinMixer");

    //Show docks
    viewer.showLeftDock();
    viewer.showRightDock();

    //Add canvas
    nvl::QGLViewerCanvas canvas(&viewer);
    viewer.addCanvas(&canvas);

    //Right dock
    nvl::DrawableListWidget drawableListWidget(&canvas);
    viewer.addToRightDock(&drawableListWidget);
    nvl::DrawableWidget drawableWidget(&canvas, &drawableListWidget);
    viewer.addToRightDock(&drawableWidget);
    nvl::ModelDrawerWidget meshDrawerWidget(&canvas, &drawableListWidget);
    viewer.addToRightDock(&meshDrawerWidget);
    nvl::SkeletonJointListWidget skeletonWidget(&canvas, &drawableListWidget);
    viewer.addToRightDock(&skeletonWidget);
    nvl::AnimationWidget animationWidget(&canvas);
    viewer.addToRightDock(&animationWidget);
    nvl::ModelAnimationWidget modelAnimationWidget(&canvas, &drawableListWidget);
    viewer.addToRightDock(&modelAnimationWidget);

    //Left dock
    SkinMixerManager skinMixerManager(&canvas, &drawableListWidget, &skeletonWidget, &modelAnimationWidget);
    viewer.addToLeftDock(&skinMixerManager);

    //Add models
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Timmy/timmy.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Crypto/crypto.rig");

//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Beagle/beagle.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Horse/horse.rig");

//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Zlorp/zlorp.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Warrok/warrok.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Dreyar/dreyar.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/AlienSoldier/alienSoldier.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/GasMask/gasMask.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/MkHuman1/mkHuman1.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/MkHuman2/mkHuman2.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/PeasantGirl/peasantGirl.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Shark/shark.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Goliath Spider/goliath_spider_rig.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/African Elephant/African_elephant.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Green Lizard/green_lizard.rig");
    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Tucano/tucano.rig");
//    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Penguin/penguin.rig");
    skinMixerManager.loadModelFromFile("/mnt/DATA/Workspace/Dataset/SkinMixer/Bat/bat.rig");

    canvas.fitScene();

    viewer.showMaximized();

    return app.exec();
}
