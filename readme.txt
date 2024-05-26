一、运行nav2给出的示例
1.运行建图 并保存地图/home/zs/my_map.pgm

2.运行gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3.运行navigation2 并使用保存的地图
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/home/zs/my_map.yaml 

4.操作rviz2 选取初始位置方向，再选取目标位置方向，完成全局路径规划及运动





二、使用自定义全局路径规划
1.新建ros2包 将nav2示例文件添加入工程

2.修改straight_line_planner.hpp文件中
config函数的参数改为rclcpp_lifecycle::LifecycleNode::SharedPtr parent

3.colcon build

4.source install/setup.bash

5.修改/opt/ros/foxy/share/nav2_bringup/params/nav2_params.yaml文件，将planner_server改为
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_straightline_planner/StraightLine"
      interpolation_resolution: 0.1

6.修改/opt/ros/foxy/share/nav2_bringup/worlds/waffle.model文件 改为tire.dae
<visual name="wheel_right_visual">
  <pose>0.0 -0.144 0.023 0 0 0</pose>
  <geometry>
    <mesh>
      <uri>model://turtlebot3_waffle/meshes/tire.dae</uri>
      <scale>0.001 0.001 0.001</scale>
    </mesh>
  </geometry>
</visual>

<visual name="wheel_left_visual">
  <pose>0.0 -0.144 0.023 0 0 0</pose>
  <geometry>
    <mesh>
      <uri>model://turtlebot3_waffle/meshes/tire.dae</uri>
      <scale>0.001 0.001 0.001</scale>
    </mesh>
  </geometry>
</visual>

7.source /opt/ros/foxy/setup.bash
8.export TURTLEBOT3_MODEL=waffle
9.export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
10.cd Desktop/ws/
11.colcon build
12.source install/setup.bash
13.ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/opt/ros/foxy/share/nav2_bringup/params/nav2_params.yaml use_sim_time:=true

