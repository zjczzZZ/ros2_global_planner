#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>

// #include "nav2_astarline_planner/astar_line_planner.hpp"
#include "../include/nav2_astarline_planner/astar_line_planner.hpp"

namespace nav2_astarline_planner {

void AStarLine::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution",
        rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution",
                         interpolation_resolution_);
}

void AStarLine::cleanup() {
    RCLCPP_INFO(node_->get_logger(),
                "CleaningUp plugin %s of type NavfnPlanner", name_.c_str());
}

void AStarLine::activate() {
    RCLCPP_INFO(node_->get_logger(),
                "Activating plugin %s of type NavfnPlanner", name_.c_str());
}

void AStarLine::deactivate() {
    RCLCPP_INFO(node_->get_logger(),
                "Deactivating plugin %s of type NavfnPlanner", name_.c_str());
}

nav_msgs::msg::Path
AStarLine::createPlan(const geometry_msgs::msg::PoseStamped &start,
                      const geometry_msgs::msg::PoseStamped &goal) {
    nav_msgs::msg::Path global_path;

    astar_.setCostmap(costmap_);

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Planner will only except start position from %s frame",
                     global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
        RCLCPP_INFO(node_->get_logger(),
                    "Planner will only except goal position from %s frame",
                    global_frame_.c_str());
        return global_path;
    }

    unsigned int mx_start, my_start, mx_goal, my_goal;

    // 设置开始位置
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start,
                         my_start);
    // 设置目标位置
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal,
                         my_goal);

    std::vector<Eigen::Vector2i> grid_path;

    if (astar_.createPath(Eigen::Vector2i(mx_start, my_start),
                          Eigen::Vector2i(mx_goal, my_goal), grid_path) &&
        !grid_path.empty()) {

        for (int i = 0; i < grid_path.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose = getWorldCoords(grid_path[i], costmap_);

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            pose.pose.position.z = 0.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);

    return global_path;
}

geometry_msgs::msg::Pose
AStarLine::getWorldCoords(const Eigen::Vector2i &pose,
                          const nav2_costmap_2d::Costmap2D *costmap) {

    geometry_msgs::msg::Pose msg;

    msg.position.x =
        static_cast<float>(costmap->getOriginX()) +
        (static_cast<float>(pose(0)) + 0.5) * costmap->getResolution();
    msg.position.y =
        static_cast<float>(costmap->getOriginY()) +
        (static_cast<float>(pose(1)) + 0.5) * costmap->getResolution();

    return msg;
}
} // namespace nav2_astarline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_astarline_planner::AStarLine,
                       nav2_core::GlobalPlanner)
