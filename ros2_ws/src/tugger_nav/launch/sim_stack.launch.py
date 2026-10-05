"""Warehouse simulator + Nav2 (planner, controller, bt_navigator) + protective_field_monitor + VDA 5050 bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("tugger_nav")
    params = os.path.join(share, "config", "nav2_params.yaml")
    field = os.path.join(share, "config", "field_monitor.yaml")
    bt_dir = os.path.join(share, "behavior_trees")
    sim_time = {"use_sim_time": True}
    log = LaunchConfiguration("log_level")
    args = ["--ros-args", "--log-level", log]

    def nav2(pkg, exe, name, remap=(), extra=None):
        p = [params, sim_time] + ([extra] if extra else [])
        return Node(package=pkg, executable=exe, name=name, output="screen", parameters=p,
                    remappings=list(remap), arguments=args)

    return LaunchDescription([
        DeclareLaunchArgument("field_monitor", default_value="true", description="false = observe only (Nav2 alone)"),
        DeclareLaunchArgument("k_approach", default_value="0.0"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        Node(package="tugger_nav", executable="warehouse_sim", name="warehouse_sim", output="screen", arguments=args),
        nav2("nav2_map_server", "map_server", "map_server",
             extra={"yaml_filename": os.path.join(share, "maps", "warehouse.yaml")}),
        nav2("nav2_planner", "planner_server", "planner_server"),
        nav2("nav2_controller", "controller_server", "controller_server", remap=[("cmd_vel", "cmd_vel_nav")]),
        nav2("nav2_behaviors", "behavior_server", "behavior_server"),
        nav2("nav2_bt_navigator", "bt_navigator", "bt_navigator", extra={
            "default_nav_to_pose_bt_xml": os.path.join(bt_dir, "to_pose.xml"),
            "default_nav_through_poses_bt_xml": os.path.join(bt_dir, "through_poses.xml")}),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", arguments=args,
             parameters=[sim_time, {"autostart": True, "bond_timeout": 0.0, "node_names": [
                 "map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]}]),
        Node(package="tugger_nav", executable="protective_field_monitor", name="protective_field_monitor",
             output="screen", arguments=args,
             parameters=[field, {"enabled": ParameterValue(LaunchConfiguration("field_monitor"), value_type=bool),
                                 "k_approach": ParameterValue(LaunchConfiguration("k_approach"), value_type=float)}]),
        Node(package="tugger_nav", executable="vda5050_bridge", name="vda5050_bridge", output="screen",
             arguments=args, parameters=[sim_time]),
    ])
