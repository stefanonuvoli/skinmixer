#include <QApplication>

#include <nvl/viewer/viewerwindow.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/drawable_widget.h>
#include <nvl/viewer/widgets/skeleton_joint_list_widget.h>
#include <nvl/viewer/widgets/animation_widget.h>
#include <nvl/viewer/widgets/model_animation_widget.h>
#include <nvl/viewer/widgets/model_drawer_widget.h>
#include <nvl/viewer/widgets/qglviewer_canvas.h>

#include "widgets/skinmixer_manager.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    //Use "." as decimal separator
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");

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

    canvas.fitScene();

    viewer.showMaximized();

    return app.exec();
}
