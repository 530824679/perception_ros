
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

void Render::RenderBBox(pcl_util::PCLVisualizerPtr &viewer, BBox box, int id, Color color){
    std::string cube = "box" + std::to_string(id);
    std::string line = "line" + std::to_string(id);
    std::string text = "text" + std::to_string(id);
    Eigen::Vector3f bboxTransform(box.x, box.y, box.z);
    Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();
    matrix << -cos(box.yaw), sin(box.yaw), 0,
                        -sin(box.yaw), -cos(box.yaw), 0,
                        0, 0, 1;
    Eigen::Quaternionf bboxQuaternion(matrix);
    std::string yaw=std::to_string(box.yaw);

    pcl::PointXYZ startPoint;
    startPoint.x=box.x;
    startPoint.y=box.y;
    startPoint.z=box.z;

    pcl::PointXYZ endPoint;
    endPoint.x=box.x+3*cos(box.yaw);
    endPoint.y=box.y+3*sin(box.yaw);
    endPoint.z=box.z;

    viewer->addCube(bboxTransform, bboxQuaternion, box.dx, box.dy, box.dz, cube);
    viewer->addLine(startPoint,endPoint,200,10,20,line);
    viewer->addText(yaw,box.x,box.y,1.0,1.0,1.0,1.0,text,0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
}

void Render::RenderTrackBBox(pcl_util::PCLVisualizerPtr &viewer, InfoTracker box, int id, Color color){
    std::string cube = "box" + std::to_string(id);
    std::string line = "line" + std::to_string(id);
    std::string text = "text" + std::to_string(id);
    Eigen::Vector3f bboxTransform(box.x, box.y, box.z);
    Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();
    matrix << -cos(box.yaw), sin(box.yaw), 0,
                        -sin(box.yaw), -cos(box.yaw), 0,
                        0, 0, 1;
    Eigen::Quaternionf bboxQuaternion(matrix);

    pcl::PointXYZ startPoint;
    startPoint.x=box.x;
    startPoint.y=box.y;
    startPoint.z=box.z;

    pcl::PointXYZ endPoint;
    endPoint.x=box.x+3*cos(box.yaw);
    endPoint.y=box.y+3*sin(box.yaw);
    endPoint.z=box.z;
    
    std::string text_show="id:"+std::to_string(box.id);
    viewer->addCube(bboxTransform, bboxQuaternion, box.width, box.length, box.height, cube);
    viewer->addLine(startPoint,endPoint,134,121,140,line);
    viewer->addText3D(text_show,startPoint,0.8,123,22,44,text);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
}

// void Render::RenderUkfTrackBBox(pcl_util::PCLVisualizerPtr &viewer, perception_ros::DetectedObjectArray box, int id, Color color){
//     std::string cube = "box" + std::to_string(id);
//     Eigen::Vector3f bboxTransform(box.pose.position.x, box.pose.position.y, box.position.z);
//     Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();
//     matrix << cos(box.yaw), sin(box.yaw), 0,
//                         -sin(box.yaw), cos(box.yaw), 0,
//                         0, 0, 1;
//     Eigen::Quaternionf bboxQuaternion(matrix);
//     viewer->addCube(bboxTransform, bboxQuaternion, box.dimension.width, box.dimension.length, box.dimension.height, cube);
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), cube);
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
// }



void Render::RenderBBox2D(pcl_util::PCLVisualizerPtr &viewer, BBox2D box2d, int id, Color color){
    std::string line = "line" + std::to_string(id);
    
    // Eigen::Vector4f top_left;
    // Eigen::Vector3f bottom_right;
    // top_left<<box2d.left_top_x,box2d.left_top_y,0;
    // bottom_right<<box2d.right_top_x,box2d.right_top_y,0;
    viewer->addLine(pcl::PointXYZ(box2d.left_top_x,box2d.left_top_y,0),pcl::PointXYZ(box2d.right_top_x,box2d.right_top_y,0),line);
    //viewer->addArrow(pcl::PointXYZ(box2d.left_top_x,box2d.left_top_y,0),pcl::PointXYZ(box2d.right_top_x,box2d.right_top_y,0),line);
    //viewer->addLine(top_left,bottom_right,line);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, line);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), line);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, line);
}