#include "db_reader.h"
#include "voxel_octree.h"
#include <iostream>
#include <memory>
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <utility>

int main()
{
    radar::DbReader reader;
    reader.connect();

    pcl::PointCloud<pcl::PointXYZ> buffer;

    // 可视化相关配置
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(500.0);
    viewer->addPointCloud(buffer.makeShared(), "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    viewer->setCameraPosition(17000, -1000, 40000, 17000, -1000, 0, 10, 0, 0, 0); //摄像机位置

    viewer->addPointCloud(pcl::PointCloud<pcl::PointXYZ>().makeShared(), "voxel cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel cloud");
    
    // 一些状态量
    bool is_paused = false;
    bool show_original = true;
    // Tested Size: -9400<y<6200
    // 键盘回调函数
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
        else if (event.getKeySym() == "v")
        {
            std::cout << "Show Original." << std::endl;
            show_original = !show_original;
            if (show_original)
                viewer->addPointCloud(buffer.makeShared(), "sample cloud");
            else
                viewer->removePointCloud("sample cloud");
        }
    });

    // 点计数
    int cnt = 0;
    float resolution = 400.0f;
    // radar::VoxelOctree<pcl::PointXYZ> last_voxel(resolution, {0, 0, 0});
    while (reader.available())
    {
        if (!is_paused)
        {
            if (cnt >= 20000)
            {
                radar::VoxelOctree<pcl::PointXYZ> octree(resolution, {0, 0, 0});
                octree.push_pointcloud(buffer);
                auto voxel_pc = octree.get_corner_points();
                viewer->updatePointCloud(
                    voxel_pc.makeShared(),
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(voxel_pc.makeShared(), 255, 255, 0),
                    "voxel cloud");
                // auto diff = radar::diff_voxel<pcl::PointXYZ>(octree, last_voxel);
                // viewer->updatePointCloud(
                //     voxel_pc.makeShared(),
                //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(voxel_pc.makeShared(), 0, 255, 255),
                //     "voxel diff");
                // last_voxel = std::move(octree);
                std::cout << "Create Voxel." << std::endl;
                // buffer.clear();
                // viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
                cnt = 0;
                is_paused = true;
            }
            else
            {
                auto points = reader.receive();
                buffer += points;
                viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
                cnt += points.size();
            }
        }
        viewer->spinOnce(10);
        // viewer->spin();
    }
    return 0;
}
