#include "db_reader.hpp"
#include <iostream>
#include <memory>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

int main()
{
    db::DbReader reader;
    reader.connect();

    pcl::PointCloud<pcl::PointXYZ> buffer;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(500.0);
    viewer->addPointCloud(buffer.makeShared(), "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    viewer->setCameraPosition(17000, -1000, 40000, 17000, -1000, 0, 10, 0, 0, 0);

    while (reader.available())
    {
        auto points = reader.receive();
        std::cout << "Received " << points.size() << " points" << std::endl;
        // viewer->updatePointCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(points), "sample cloud", 0);
        buffer += points;
        viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
        viewer->spinOnce(10);
        // viewer->spin();
    }
    return 0;
}
