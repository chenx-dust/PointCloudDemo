#include "ground_rm.h"
#include "voxel_octree.h"
#include "voxel_op.h"
#include <iostream>
#include <memory>
#include <pcl/point_types.h>
#include <vector>

namespace radar
{

int get_x_floor(float x)
{
    return std::ceil(x / X_FLOOR_RESO) * X_FLOOR_RESO;
}

void GroundNode::_after_push_point(const pcl::PointXYZ &point)
{
    if (contained_points < static_cast<size_t>(-1))
        contained_points++;
    is_updated = true;
}

void GroundNode::refresh(bool is_contructing, size_t ref_points)
{
    float k;
    if (is_updated)
        k = is_contructing ? GROUND_TRUE_K_CON : GROUND_TRUE_K;
    else
    {
        if (is_contructing)
            k = -GROUND_FALSE_K_CON;
        else
        {
            if (contained_points > ref_points)
                k = -GROUND_FALSE_K_REF;
            else
                k = -GROUND_FALSE_K;
        }
    }
    _update_safely(intensity, intensity + k);
    is_updated = false;
}

void GroundVoxel::update(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    push_pointcloud(cloud, [&](const GroundNode &node, bool is_new) { push_callback(node, is_new); });
}

void GroundVoxel::traverse(const std::function<void(GroundNode &)> func)
{
    if (root)
        voxel_op::recursive_traverse<GroundNode>(dynamic_cast<GroundNode &>(*root), func);
}

void GroundVoxel::traverse_idx(const std::function<void(GroundNode &, const std::vector<size_t>&)> func)
{
    if (root)
        voxel_op::recursive_traverse_idx<GroundNode>(dynamic_cast<GroundNode &>(*root), func);
}

/// \brief 添加点的回调函数
/// \details 用于更新 x_points 和 x_voxels
void GroundVoxel::push_callback(const GroundNode &node, bool is_new)
{
    if (node.size != 0)
        return;
    float x_floor = get_x_floor(node.location.x);
    if (is_new)
    {
        if (!x_voxels.contains(x_floor))
            x_voxels[x_floor] = 1;
        else
            x_voxels[x_floor]++;
    }
    if (!x_points.contains(x_floor))
        x_points[x_floor] = 1;
    else
        x_points[x_floor]++;
}

void GroundVoxel::refresh()
{
    auto is_contructing = this->is_contructing;
    traverse([&](GroundNode &node) {
        if (node.size != 0)
            return;
        int x_floor = get_x_floor(node.location.x);
        node.refresh(is_contructing, this->x_points[x_floor] / (this->x_voxels[x_floor] > 0 ? this->x_voxels[x_floor] : 1));
    });
}

pcl::PointCloud<pcl::PointXYZI> GroundVoxel::get_corner_points_with_i()
{
    pcl::PointCloud<pcl::PointXYZI> pc;
    traverse([&pc](GroundNode &node) {
        if (node.size == 0)
            pc.push_back(pcl::PointXYZI(node.location.x, node.location.y, node.location.z, node.intensity));
    });
    return pc;
}

pcl::PointCloud<pcl::PointXYZRGB> GroundVoxel::get_corner_points_with_rgb()
{
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    traverse([&pc](GroundNode &node) {
        if (node.size == 0)
            pc.push_back(pcl::PointXYZRGB(node.location.x, node.location.y, node.location.z,
                                          node.intensity * 255, node.intensity * 255, node.intensity * 255));
    });
    return pc;
}

pcl::PointCloud<pcl::PointXYZ> GroundVoxel::get_real_ground()
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    traverse([&pc](GroundNode &node) {
        if (node.size == 0 && node.intensity > GROUND_THRESHOLD)
            pc.push_back(node.location);
    });
    return pc;
}

bool GroundVoxel::has_index(const std::vector<size_t> &idx) const
{
    std::shared_ptr<GroundNode> node = root;
    for (auto i : idx)
    {
        if (node->children[i])
            auto new_node = node->children[i];
        else
            return false;
    }
    return true;
}

bool _is_ground_helper(GroundNode &node, const pcl::PointXYZ &point)
{
    if (voxel_op::is_contain<GroundNode, pcl::PointXYZ>(node, point))
    {
        if (node.size > 0)
        {
            float fsize = std::pow(2, node.size - 1);
            bool x_flag = point.x >= node.location.x + fsize * node.resolution.x;
            bool y_flag = point.y >= node.location.y + fsize * node.resolution.y;
            bool z_flag = point.z >= node.location.z + fsize * node.resolution.z;
            int index = (x_flag << 2) + (y_flag << 1) + z_flag;
            if (!node.children[index])
                return false;
            return _is_ground_helper(*node.children[index], point);
        }
        else
            return node.intensity > GROUND_THRESHOLD;
    }
    else
        return false;
}

pcl::PointCloud<pcl::PointXYZ> GroundVoxel::filter_ground(const pcl::PointCloud<pcl::PointXYZ> &cloud) const
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (auto &point : cloud.points)
    {
        if (!_is_ground_helper(*root, point))
            pc.push_back(point);
    }
    return pc;
}

} // namespace radar
