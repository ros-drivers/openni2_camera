import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name)

    tf_prefix_param_name = "tf_prefix"
    tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name)

    tf_args = [
      ["--frame-id", [tf_prefix,"/",namespace,"_link"],
       "--child-frame-id", [tf_prefix,"/",namespace,"_depth_frame"],
       "--y", "-0.02"],
      ["--frame-id", [tf_prefix,"/",namespace,"_link"],
       "--child-frame-id", [tf_prefix,"/",namespace,"_rgb_frame"],
       "--y", "-0.045"],
      ["--frame-id", [tf_prefix,"/",namespace,"_depth_frame"],
       "--child-frame-id", [tf_prefix,"/",namespace,"_depth_optical_frame"],
       "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
      ["--frame-id", [tf_prefix,"/",namespace,"_rgb_frame"],
       "--child-frame-id", [tf_prefix,"/",namespace,"_rgb_optical_frame"],
       "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"],
    ]

    tf_nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='screen', arguments=args) for args in tf_args]

    return launch.LaunchDescription([namespace_launch_arg, tf_prefix_launch_arg] + tf_nodes)
