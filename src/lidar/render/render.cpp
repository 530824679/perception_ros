
// local include
#include "render/render.h"

Render::Render() {

}

Render::~Render() {

}

void Render::InitCamera(CameraAngle angle, pcl_util::PCLVisualizerPtr &viewer) {
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    int distance = 16;
    switch(angle){
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(angle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void Render::RenderPointCloud(pcl_util::PCLVisualizerPtr &viewer, const pcl_util::PointCloudPtr &cloud, std::string name, Color color) {
    viewer->addPointCloud<pcl_util::Point> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name);
}