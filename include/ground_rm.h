#pragma once

#include "voxel_octree.h"
#include "voxel_op.h"
#include <map>
#include <memory>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Configs
// 根据是否在建图, 选择不同的更新速率
// 建图时, 更信任新点
// 非建图时, 更信任旧点
#define GROUND_TRUE_K 0.05
#define GROUND_FALSE_K 0.2
#define GROUND_TRUE_K_CON 1.0
#define GROUND_FALSE_K_CON 0.1
#define GROUND_FALSE_K_REF 0.05
#define GROUND_THRESHOLD 0.5

#define X_FLOOR_RESO 200
#define VOXEL_EXTEND_RATE 0.5

#define _update_safely(key, value) (key) = (((value) <= 1.0) ? (((value) >= 0.0) ? (value) : 0.0) : 1.0)

namespace radar
{
int get_x_floor(float x);

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
    std::shared_ptr<GroundNode> children[8];
    const pcl::PointXYZ location;

    void _after_push_point(const pcl::PointXYZ &point);
    GroundNode(size_t size, pcl::PointXYZ resolution, pcl::PointXYZ location)
        : size(size), resolution(resolution), location(location) {}
    /// \brief 定时刷新, 更新强度
    /// \details 一般而言是一个扫描周期
    /// \param is_contructing 是否在建图
    void refresh(bool is_contructing, size_t ref_points);
};

class GroundVoxel : public VoxelOctree<GroundNode, pcl::PointXYZ>
{
  private:
    using VoxelOctree<GroundNode, pcl::PointXYZ>::push_pointcloud;
    using VoxelOctree<GroundNode, pcl::PointXYZ>::push_point;
    bool is_contructing = false;
    std::map<int, size_t> x_points;
    std::map<int, size_t> x_voxels;

  public:
    GroundVoxel(pcl::PointXYZ resolution)
        : VoxelOctree<GroundNode, pcl::PointXYZ>(resolution, pcl::PointXYZ(0, 0, 0)) {}
    void update(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    void traverse(std::function<void(GroundNode &)>);
    void traverse_idx(std::function<void(GroundNode &, const std::vector<size_t> &)>);
    void refresh();
    void set_is_contructing(bool value) { is_contructing = value; }
    pcl::PointCloud<pcl::PointXYZI> get_corner_points_with_i();
    pcl::PointCloud<pcl::PointXYZRGB> get_corner_points_with_rgb();
    pcl::PointCloud<pcl::PointXYZ> get_real_ground();
    bool has_index(const std::vector<size_t> &index) const;
    void push_callback(const GroundNode &node, bool);
    pcl::PointCloud<pcl::PointXYZ> filter_ground(const pcl::PointCloud<pcl::PointXYZ> &cloud) const;
};

// template <typename PointT>
// class VoxelWithPointsNode : public VoxelNode<PointT>
// {
//   public:
//     std::vector<std::shared_ptr<PointT>> points;
//     std::shared_ptr<VoxelWithPointsNode<PointT>> children[8];
//     VoxelWithPointsNode(size_t size, pcl::PointXYZ resolution, pcl::PointXYZ location)
//         : VoxelNode<PointT>(size, resolution, location) {}
//     void _after_push_point(const PointT &point)
//     {
//         points.push_back(pcl::make_shared<PointT>(point));
//     }
// };

// template <typename PointT>
// pcl::PointCloud<PointT> diff_with_ground(const GroundVoxel &ground, const pcl::PointCloud<PointT> &cloud)
// {
//     pcl::PointCloud<PointT> result;
//     VoxelOctree<VoxelWithPointsNode<PointT>, PointT> voxel(ground.resolution, pcl::PointXYZ(0, 0, 0));
//     voxel.push_pointcloud(cloud);

//     voxel.traverse_idx([&](VoxelWithPointsNode<PointT> &node, const std::vector<size_t> &idx) {
//         if (node.size == 0 && !ground.has_index(idx))
//             for (auto &point : node.points)
//                 result.push_back(*point);
//     });
//     return result;
// }

}
