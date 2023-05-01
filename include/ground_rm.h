#pragma once

#include "voxel_octree.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar
{

class GroundNode
{
  public:
    /// \brief 强度
    /// \details 用于判断是否为地面点
    float intensity = 0.0;
    size_t contained_points = 0; // 留意会不会爆
    bool is_updated = false;

    const size_t size = 0; // 0 = leaf node
    const pcl::PointXYZ resolution;
    std::unique_ptr<GroundNode> children[8];
    const pcl::PointXYZ location;

    void _after_push_point(const pcl::PointXYZ &point);
    GroundNode(size_t size, pcl::PointXYZ resolution, pcl::PointXYZ location)
        : size(size), resolution(resolution), location(location)
    {
    }
    /// \brief 定时刷新, 降低强度
    void flush();
};

class GroundVoxel : public VoxelOctree<GroundNode, pcl::PointXYZ>
{
  private:
    using VoxelOctree<GroundNode, pcl::PointXYZ>::push_pointcloud;
    using VoxelOctree<GroundNode, pcl::PointXYZ>::push_point;
  public:
    GroundVoxel(pcl::PointXYZ resolution)
        : VoxelOctree<GroundNode, pcl::PointXYZ>(resolution, pcl::PointXYZ(0, 0, 0)) {}
    void update(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    void traverse(std::function<void(GroundNode &)>);
    // pcl::PointCloud<pcl::PointXYZI> get_corner_points();
};
}
