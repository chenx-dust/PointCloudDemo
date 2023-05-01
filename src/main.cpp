#include "db_reader.h"
#include "ground_rm.h"
#include "voxel_octree.h"
#include "voxel_op.h"
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
#include <vector>

int main()
{
    radar::DbReader reader;
    reader.connect();

    pcl::PointCloud<pcl::PointXYZ> all_points, buffer;

    int v0 = 0, v1 = 0;
    
    // 可视化相关配置
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v0);
    viewer->setBackgroundColor(0, 0, 0, v0);
    viewer->addCoordinateSystem(1000.0, "cs", v0);
    viewer->addPointCloud(all_points.makeShared(), "sample cloud", v0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud", v0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(17000, -1000, 40000, 17000, -1000, 0, 10, 0, 0, v0); //摄像机位置

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v1);
    viewer->addCoordinateSystem(1000.0, "cs", v0);
    viewer->setCameraPosition(17000, -1000, 40000, 17000, -1000, 0, 10, 0, 0, v1);

    viewer->addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>().makeShared(), "voxel cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "voxel cloud", v1);

    viewer->addPointCloud(pcl::PointCloud<pcl::PointXYZ>().makeShared(), "voxel ground", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "voxel ground", v1);

    viewer->addPointCloud(pcl::PointCloud<pcl::PointXYZ>().makeShared(), "diff ground", v0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "diff ground", v0);

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
            all_points.clear();
            viewer->updatePointCloud(all_points.makeShared(), "sample cloud");
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
                viewer->addPointCloud(all_points.makeShared(), "sample cloud", v0);
            else
                viewer->removePointCloud("sample cloud", v0);
        }
    });

    // 点计数
    int cnt = 0, tot = 0;
    pcl::PointXYZ resolution(400, 200, 200);
    // radar::VoxelOctree<pcl::PointXYZ> last_voxel(resolution, {0, 0, 0});
    radar::GroundVoxel ground(resolution);
    ground.set_is_contructing(true);
    while (reader.available())
    {
        if (!is_paused)
        {
            if (cnt >= 20000)
            {
                ground.refresh();

                if (tot > 200000)
                {
                    ground.set_is_contructing(false);
                    std::cout << "Stop Constructing." << std::endl;
                }
                auto voxel_pc = ground.get_corner_points_with_rgb();
                // radar::VoxelOctree<radar::VoxelNode<pcl::PointXYZ>, pcl::PointXYZ> voxel(resolution, {0, 0, 0});
                // voxel.push_pointcloud(all_points);
                // auto voxel_pc = voxel.get_corner_points();
                viewer->updatePointCloud(
                    voxel_pc.makeShared(),
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(voxel_pc.makeShared()),
                    "voxel cloud");
                auto ground_pc = ground.get_real_ground();
                viewer->updatePointCloud(ground_pc.makeShared(),
                                         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
                                             ground_pc.makeShared(), 255, 255, 0),
                                         "voxel ground");
                // auto diff = radar::diff_with_ground(ground, all_points);
                auto diff = ground.filter_ground(all_points);
                viewer->updatePointCloud<pcl::PointXYZ>(diff.makeShared(),
                                         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(
                                             ground_pc.makeShared(), 255, 0, 0),
                                         "diff ground");

                std::cout << "Create Voxel. At " << tot << std::endl;
                buffer.clear();
                // viewer->updatePointCloud(buffer.makeShared(), "sample cloud");
                // ground.traverse([](radar::GroundNode &node) {
                //     if (node.size == 0)
                //         cout << "leaf with " << node.contained_points << endl;
                // });
                // ground.traverse_idx([](radar::GroundNode &node, const std::vector<size_t> index) {
                //     // if (node.size != 0)
                //     //     return;
                //     for (auto i : index)
                //         cout << i << " ";
                //     cout << "size:"<< node.size << endl;
                // });
                cnt = 0;
                is_paused = true;
            }
            else
            {
                auto points = reader.receive();
                all_points += points;
                buffer += points;
                ground.update(points);
                viewer->updatePointCloud(all_points.makeShared(), "sample cloud");
                cnt += points.size();
                tot += points.size();
            }
        }
        viewer->spinOnce(10);
        // viewer->spin();
    }
    return 0;
}
