#include <iostream>
#include <memory>
#include <cmath>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#define SQ(x) ((x) * (x))

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile("../test_data/hap-jicheng-kongchang.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    int skip_cnt = 0;
    for (auto p : cloud->points)
    {
        skip_cnt++;
        if (skip_cnt % 10 != 0)
            continue;
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p.x;
        p_rgb.y = p.y;
        p_rgb.z = p.z;
        uint8_t dis = std::sqrt(SQ(p.x) + SQ(p.y) + SQ(p.z)) * 10;
        p_rgb.r = dis;
        p_rgb.g = dis;
        p_rgb.b = dis;
        cloud_filtered->push_back(p_rgb);
    }
    std::cout << "Filtered." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    std::cout << "Start." << std::endl;
    while (!viewer->wasStopped())
        viewer->spinOnce(100);

    return 0;
}