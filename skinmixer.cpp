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
    nvl::DrawableWidget drawableWidget(&canvas);
    viewer.addToRightDock(&drawableWidget);
    nvl::ModelDrawerWidget meshDrawerWidget(&canvas);
    viewer.addToRightDock(&meshDrawerWidget);
    nvl::SkeletonJointListWidget skeletonWidget(&canvas);
    viewer.addToRightDock(&skeletonWidget);
    nvl::AnimationWidget animationWidget(&canvas);
    viewer.addToRightDock(&animationWidget);
    nvl::ModelAnimationWidget modelAnimationWidget(&canvas);
    viewer.addToRightDock(&modelAnimationWidget);

    //Left dock
    skinmixer::SkinMixerManager skinMixerManager(&canvas);
    viewer.addToLeftDock(&skinMixerManager);


//    for (const auto& joint : model.skeleton.joints()) {
//        nvl::Color color = nvl::getRangeDifferentColor(maxLabel + 1, static_cast<int>(joint.id()));
//        modelDrawer.skeletonDrawer().setRenderingJointColor(joint.id(), color);
//    }

    viewer.showMaximized();

    return app.exec();
}

