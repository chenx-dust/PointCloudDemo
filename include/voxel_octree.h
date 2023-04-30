#pragma once

#include <functional>
#include <cmath>
#include <iostream>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unistd.h>
namespace radar
{

template <typename PointT>
class VoxelNode
{
  protected:
    void _leaf_push_point(PointT p)
    {
        // std::cout << "Pushed " << p << " in " << location << std::endl;
    };
    
  public:
    const size_t size = 0; // 0 = leaf node
    const float resolution;
    std::shared_ptr<VoxelNode<PointT>> children[8];
    pcl::PointXYZ location;
    bool push_point(PointT &point);
    bool is_contain(PointT &point);
    void recursive_traverse(std::function<void(VoxelNode<PointT> &)>);
    VoxelNode(size_t size, float resolution) : size(size), resolution(resolution) {}
    VoxelNode(size_t size, float resolution, pcl::PointXYZ location) : size(size), resolution(resolution), location(location) {}
};

template <typename PointT>
class VoxelOctree
{
  private:
    void _upgrade_recursive(PointT &p);
  protected:
    std::shared_ptr<VoxelNode<PointT>> root = nullptr;
  public:
    const float resolution;
    VoxelOctree(float resolution) : resolution(resolution) {}
    VoxelOctree(float resolution, pcl::PointXYZ location)
        : resolution(resolution), root(std::shared_ptr<VoxelNode<PointT>>(new VoxelNode<PointT>(0, resolution, location))) {}
    void push_pointcloud(pcl::PointCloud<PointT> &pc);
    void push_point(PointT &p);
    /// \brief 获取所有子节点的角点
    pcl::PointCloud<pcl::PointXYZ> get_corner_points();
    /// \brief 遍历所有节点
    void traverse(std::function<void(VoxelNode<PointT> &)>);
};

template <typename PointT> bool VoxelNode<PointT>::push_point(PointT &p)
{
    if (is_contain(p))
    {
        if (size > 0)
        {
            float fsize = resolution * std::pow(2, size - 1);
            bool x_flag = p.x >= location.x + fsize;
            bool y_flag = p.y >= location.y + fsize;
            bool z_flag = p.z >= location.z + fsize;
            int index = (x_flag << 2) + (y_flag << 1) + z_flag;
            if (!children[index])
            {
                children[index] = std::shared_ptr<VoxelNode<PointT>>(new VoxelNode<PointT> (size - 1, resolution));
                children[index]->location = pcl::PointXYZ(location.x + (x_flag ? fsize : 0),
                                                          location.y + (y_flag ? fsize : 0),
                                                          location.z + (z_flag ? fsize : 0));
            }
            return children[index]->push_point(p);
        }
        else
        {
            _leaf_push_point(p);
            return true;
        }
    }
    else
        return false;
}

template <typename PointT> bool VoxelNode<PointT>::is_contain(PointT &point)
{
    float fsize = resolution * std::pow(2, size);
    float x = location.x;
    float y = location.y;
    float z = location.z;
    return point.x >= x && point.x < x + fsize && point.y >= y && point.y < y + fsize && point.z >= z &&
           point.z < z + fsize;
}

template <typename PointT> void VoxelNode<PointT>::recursive_traverse(std::function<void(VoxelNode<PointT> &)> func)
{
    func(*this);
    if (size > 0)
        for (auto &child : children)
            if (child)
                child->recursive_traverse(func);
}

template <typename PointT> void VoxelOctree<PointT>::push_point(PointT &p)
{
    if (!root)
    {
        root = std::shared_ptr<VoxelNode<PointT>>(new VoxelNode<PointT>(0, resolution));
        float x = std::ceil(p.x / resolution) * resolution;
        float y = std::ceil(p.y / resolution) * resolution;
        float z = std::ceil(p.z / resolution) * resolution;
        root->location = pcl::PointXYZ(x, y, z);
    }
    else if (!root->push_point(p))
    {
        _upgrade_recursive(p);
        push_point(p);
    }
}

template <typename PointT> void VoxelOctree<PointT>::push_pointcloud(pcl::PointCloud<PointT> &pc)
{
    for (auto p : pc)
        push_point(p);
}

template <typename PointT> pcl::PointCloud<pcl::PointXYZ> VoxelOctree<PointT>::get_corner_points()
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    traverse([&pc](VoxelNode<PointT> &node) {
        if (node.size == 0)
        {
            pc.push_back(node.location);
            pc.push_back(pcl::PointXYZ(node.location.x, node.location.y, node.location.z));
        }
    });
    return pc;
}

template <typename PointT> void VoxelOctree<PointT>::traverse(std::function<void(VoxelNode<PointT> &)> func)
{
    if (root)
        root->recursive_traverse(func);
}

template <typename PointT>
void VoxelOctree<PointT>::_upgrade_recursive(PointT &point)
{
    // std::cout << "upgrade with point: " << point << std::endl;
    bool x_flag = point.x < root->location.x;
    bool y_flag = point.y < root->location.y;
    bool z_flag = point.z < root->location.z;
    int index = (x_flag << 2) + (y_flag << 1) + z_flag;
    auto tmp_root = std::move(root);
    root = std::shared_ptr<VoxelNode<PointT>>(new VoxelNode<PointT>(tmp_root->size + 1, resolution));
    root->location = pcl::PointXYZ(tmp_root->location.x - (x_flag ? resolution * std::pow(2, tmp_root->size) : 0),
                                   tmp_root->location.y - (y_flag ? resolution * std::pow(2, tmp_root->size) : 0),
                                   tmp_root->location.z - (z_flag ? resolution * std::pow(2, tmp_root->size) : 0));
    root->children[index] = std::move(tmp_root);
    // std::cout << "upgrade to: " << root->location << "size: " << root->size << std::endl;
    if (!root->push_point(point))
        _upgrade_recursive(point);
}

// // voxel1 - voxel2
// template <typename PointT>
// pcl::PointCloud<PointT> diff_voxel(VoxelOctree<PointT> &voxel1, VoxelOctree<PointT> &voxel2)
// {
//     if (voxel1.resolution != voxel2.resolution)
//         throw std::runtime_error("resolution not equal");
//     if (voxel1.root->location != voxel2.root->location)
//         throw std::runtime_error("location not equal");

//     pcl::PointCloud<PointT> pc;
//     diff_node(*voxel1.root, *voxel2.root, pc);
//     return pc;
// }

// template <typename PointT>
// void diff_node(VoxelNode<PointT> &node1, VoxelNode<PointT> &node2, pcl::PointCloud<PointT> &pc)
// {
//     if (node1.size == 0 && node2.size != 0)
//     {
//         pc.push_back(node1.location);
//     }
//     else
//     {
//         if (node2.size == 0)
//         {
//             for (auto &child : node1.children)
//                 if (child)
//                     diff_node(*child, node2, pc);
//         }
//         else
//         {
//             for (int i = 0; i < 8; i++)
//                 if (node1.children[i] && node2.children[i])
//                     diff_node(*node1.children[i], *node2.children[i], pc);
//         }
//     }
// }
}
