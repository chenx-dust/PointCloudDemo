#include "db_reader.hpp"
#include <iostream>
#include <memory>
#include <pcl/ModelCoefficients.h>
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

    bool is_paused = false;
    int filtered_y = 0;

    // Tested Size: -9400<y<6200
    
    viewer->registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent &event) {
        if (!event.keyDown())
            return;
        if (event.getKeySym() == "space")
        {
            if (is_paused)
                std::cout << "Resumed." << std::endl;
            else
                std::cout << "Paused." << std::endl;
            is_paused = !is_paused;
        }
        else if (event.getKeySym() == "c")
        {
            std::cout << "Cleared." << std::endl;
            buffer.clear();
            viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
        }
        else if (event.getKeySym() == "s")
        {
            std::cout << "Skip." << std::endl;
            reader.skip(10000);
        }
        else if (event.getKeySym() == "a")
        {
            filtered_y -= 100;
            reader.setFilter([=](int x, int y, int z) { return y < filtered_y; });
            std::cout << "Filtered y < " << filtered_y << std::endl;
        }
        else if (event.getKeySym() == "d")
        {
            filtered_y += 100;
            reader.setFilter([=](int x, int y, int z) { return y < filtered_y; });
            std::cout << "Filtered y < " << filtered_y << std::endl;
        }
    });

    while (reader.available())
    {
        if (!is_paused)
        {
            auto points = reader.receive();
            buffer += points;
            viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
        }
        viewer->spinOnce(10);
        // viewer->spin();
    }
    return 0;
}
