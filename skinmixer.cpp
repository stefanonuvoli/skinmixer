#include <QApplication>

#include <nvl/viewer/viewerwindow.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/drawable_widget.h>
#include <nvl/viewer/widgets/skeletonjoint_list_widget.h>
#include <nvl/viewer/widgets/animation_widget.h>
#include <nvl/viewer/widgets/drawable_handler_widget.h>
#include <nvl/viewer/widgets/model_animation_widget.h>
#include <nvl/viewer/widgets/model_drawer_widget.h>
#include <nvl/viewer/widgets/qglviewer_canvas.h>

#include "widgets/skinmixer_manager.h"

#include <set>
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
    nvl::DrawableHandlerWidget drawableHandlerWidget(&canvas, &drawableListWidget);
    viewer.addToLeftDock(&drawableHandlerWidget);
    skinmixer::SkinMixerManager skinMixerManager(&canvas, &drawableListWidget, &skeletonWidget);
    viewer.addToLeftDock(&skinMixerManager);

    skinMixerManager.loadModelFromFile("/mnt/OS/Workspace/Dataset/SkinMixer/dreyar.mdl");
    canvas.fitScene();

    viewer.showMaximized();

    return app.exec();
}

