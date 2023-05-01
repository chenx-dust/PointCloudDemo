#pragma once

#include <functional>
#include <cmath>
#include <iostream>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>

#include "voxel_op.h"

namespace radar
{

template <typename PointT>
class VoxelNode
{

  public:
    const size_t size = 0; // 0 = leaf node
    const pcl::PointXYZ resolution;
    std::unique_ptr<VoxelNode<PointT> > children[8];
    const pcl::PointXYZ location;
    std::function<void(const PointT &point)> _after_push_point = [](const PointT &point) {};
    VoxelNode(size_t size, const pcl::PointXYZ &resolution, const pcl::PointXYZ &location)
        : size(size), resolution(resolution), location(location) {}
};

template <typename NodeT, typename PointT>
class VoxelOctree
{
  private:
    void _upgrade_recursive(const PointT &p);
  protected:
    std::unique_ptr<NodeT> root = nullptr;
  public:
    const pcl::PointXYZ resolution;
    VoxelOctree(pcl::PointXYZ resolution) : resolution(resolution) {}
    VoxelOctree(float resolution) : resolution(pcl::PointXYZ(resolution, resolution, resolution)) {}
    VoxelOctree(pcl::PointXYZ resolution, pcl::PointXYZ location)
        : resolution(resolution), root(std::unique_ptr<NodeT>(new NodeT(0, resolution, location))) {}
    VoxelOctree(float resolution, pcl::PointXYZ location)
        : resolution(pcl::PointXYZ(resolution, resolution, resolution)),
          root(std::unique_ptr<NodeT>(new NodeT(0, resolution, location))) {}
    void push_pointcloud(const pcl::PointCloud<PointT> &pc);
    void push_point(const PointT &p);
    /// \brief 获取所有子节点的角点
    pcl::PointCloud<pcl::PointXYZ> get_corner_points();
    /// \brief 遍历所有节点
    void traverse(std::function<void(NodeT &)>);
};


template <typename NodeT, typename PointT> void VoxelOctree<NodeT, PointT>::push_point(const PointT &p)
{
    if (!root)
    {
        float x = std::ceil(p.x / resolution.x) * resolution.x;
        float y = std::ceil(p.y / resolution.y) * resolution.y;
        float z = std::ceil(p.z / resolution.z) * resolution.z;
        root = std::unique_ptr<NodeT>(new NodeT(0, resolution, pcl::PointXYZ(x, y, z)));
    }
    else if (!voxel_op::push_point<NodeT, PointT>(*root, p))
    {
        _upgrade_recursive(p);
        push_point(p);
    }
}

template <typename NodeT, typename PointT> void VoxelOctree<NodeT, PointT>::push_pointcloud(const pcl::PointCloud<PointT> &pc)
{
    for (auto p : pc)
        push_point(p);
}

template <typename NodeT, typename PointT> pcl::PointCloud<pcl::PointXYZ> VoxelOctree<NodeT, PointT>::get_corner_points()
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    traverse([&pc](NodeT &node) {
        if (node.size == 0)
        {
            pc.push_back(node.location);
            pc.push_back(pcl::PointXYZ(node.location.x, node.location.y, node.location.z));
        }
    });
    return pc;
}

template <typename NodeT, typename PointT> void VoxelOctree<NodeT, PointT>::traverse(std::function<void(NodeT &)> func)
{
    if (root)
        voxel_op::recursive_traverse<NodeT>(*root, func);
}

template <typename NodeT, typename PointT>
void VoxelOctree<NodeT, PointT>::_upgrade_recursive(const PointT &point)
{
    // std::cout << "upgrade with point: " << point << std::endl;
    bool x_flag = point.x < root->location.x;
    bool y_flag = point.y < root->location.y;
    bool z_flag = point.z < root->location.z;
    int index = (x_flag << 2) + (y_flag << 1) + z_flag;
    auto tmp_root = std::move(root);
    root = std::unique_ptr<NodeT>(new NodeT(
        tmp_root->size + 1, resolution,
        pcl::PointXYZ(tmp_root->location.x - (x_flag ? resolution.x * std::pow(2, tmp_root->size) : 0),
                                 tmp_root->location.y - (y_flag ? resolution.y * std::pow(2, tmp_root->size) : 0),
                                 tmp_root->location.z - (z_flag ? resolution.z * std::pow(2, tmp_root->size) : 0))));
    
    root->children[index] = std::move(tmp_root);
    // std::cout << "upgrade to: " << root->location << "size: " << root->size << std::endl;
    if (!voxel_op::push_point<NodeT, PointT>(*root, point))
        _upgrade_recursive(point);
}
}
