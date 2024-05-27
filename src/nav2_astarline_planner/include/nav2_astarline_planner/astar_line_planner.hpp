#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "Astar.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_astarline_planner {

class AStarLine : public nav2_core::GlobalPlanner {
  public:
    AStarLine() {}
    ~AStarLine() {}

    // plugin configure
    virtual void configure(
        rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    virtual void cleanup() override;

    // plugin activate
    virtual void activate() override;

    // plugin deactivate
    virtual void deactivate() override;

    // This method creates path for given start and goal pose.
    virtual nav_msgs::msg::Path
    createPlan(const geometry_msgs::msg::PoseStamped &start,
               const geometry_msgs::msg::PoseStamped &goal) override;

    geometry_msgs::msg::Pose
    getWorldCoords(const Eigen::Vector2i &pose,
                   const nav2_costmap_2d::Costmap2D *costmap);

  private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D *costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    Astar astar_;

    double interpolation_resolution_;
};

} // namespace nav2_astarline_planner
