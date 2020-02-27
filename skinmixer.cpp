#include <QApplication>

#include <nvl/io/model_io.h>

#include <nvl/models/model.h>
#include <nvl/models/mesh_normals.h>
#include <nvl/models/mesh_vertex_transformations.h>

#include <nvl/viewer/viewerwindow.h>
#include <nvl/viewer/widgets/drawable_list_widget.h>
#include <nvl/viewer/widgets/drawable_widget.h>
#include <nvl/viewer/widgets/skeletonjoint_list_widget.h>
#include <nvl/viewer/widgets/model_drawer_widget.h>
#include <nvl/viewer/widgets/qglviewer_canvas.h>

#include <nvl/viewer/drawables/model_drawer.h>

#include "widgets/skinmixer_manager.h"

int main(int argc, char *argv[]) {
    typedef nvl::Model3d Model;
    typedef nvl::ModelDrawer<Model> ModelDrawer;

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

    //Left dock
    skinmixer::SkinMixerManager skinMixerManager(&canvas);
    viewer.addToLeftDock(&skinMixerManager);


    Model model;

    nvl::loadModel("/mnt/OS/Workspace/Dataset/SkinMixer/triangles/dreyar/dreyar.obj", model);
    nvl::loadModelAnimation("/mnt/OS/Workspace/Dataset/SkinMixer/triangles/dreyar/dreyar_anim_1.txt", model);

    nvl::meshUpdateFaceNormals(model.mesh);
    nvl::meshUpdateVertexNormals(model.mesh);

    ModelDrawer modelDrawer(&model);
    viewer.canvas()->addDrawable(&modelDrawer, "dreyar.obj");

    //TODO CANCEL!!!!
    ModelDrawer modelDrawer2(&model);
    viewer.canvas()->addDrawable(&modelDrawer2, "dreyar.obj");

    ModelDrawer modelDrawer3(&model);
    viewer.canvas()->addDrawable(&modelDrawer3, "dreyar.obj");

    std::unordered_set<size_t> asd;
    asd.insert(0);
    asd.insert(2);
    viewer.canvas()->setSelectedDrawables(asd);


    std::vector<double> vertexValues(model.mesh.vertexNumber(), 0);
    for (auto vertex : model.mesh.vertices()) {
        vertexValues[vertex.id()] = model.skinningWeights.weight(vertex.id(), 61);
    }
    modelDrawer.meshDrawer().setVertexValues(vertexValues);

//    for (const auto& joint : model.skeleton.joints()) {
//        nvl::Color color = nvl::getRangeDifferentColor(maxLabel + 1, static_cast<int>(joint.id()));
//        modelDrawer.skeletonDrawer().setRenderingJointColor(joint.id(), color);
//    }

    modelDrawer.meshDrawer().setFaceColorMode(ModelDrawer::FaceColorMode::FACE_COLOR_PER_FACE);
    modelDrawer.meshDrawer().setWireframeVisible(true);
    modelDrawer.skeletonDrawer().setVisible(true);


    viewer.canvas()->fitScene();

    viewer.showMaximized();

    modelDrawer.setAnimationTargetFPS(60);
    modelDrawer.setAnimationSkinningMode(ModelDrawer::SkinningMode::SKINNING_DUAL_QUATERNIONS);
    modelDrawer.setAnimationLoop(true);
    modelDrawer.setAnimationSpeed(1);
    modelDrawer.loadAnimation(0);

    viewer.canvas()->setTargetFPS(60);
    viewer.canvas()->startAnimations();

    return app.exec();
}

