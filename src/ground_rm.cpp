#include "ground_rm.h"
#include "voxel_octree.h"
#include "voxel_op.h"
#include <iostream>

namespace radar
{

void GroundNode::_after_push_point(const pcl::PointXYZ &point)
{
    if (contained_points < static_cast<size_t>(-1))
        contained_points++;
    if (is_updated)
    {
        std::cout << "Updated with" << contained_points << std::endl;
        is_updated = false;
    }
}

void GroundVoxel::update(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    push_pointcloud(cloud);
    traverse([](GroundNode &node) {
        if (node.size == 0){
            node.is_updated = true;
            // std::cout<<"Child at " << node.location << " has " << node.contained_points << std::endl;
            }
    });
}

void GroundVoxel::traverse(const std::function<void(GroundNode &)> func)
{
    if (root)
        voxel_op::recursive_traverse<GroundNode>(dynamic_cast<GroundNode&>(*root), func);
}
}
