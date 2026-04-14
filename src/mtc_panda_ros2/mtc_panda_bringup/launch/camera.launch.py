from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py'])),
            launch_arguments={
                # 'camera_namespace': 'd435',
                'align_depth.enable': 'true',
                'disparity_filter.enable': 'true',
                'spatial_filter': 'true',
                'temporal_filter': 'true',
                'decimation_filter': 'true',
                'hole_filling_filter': 'true',
                'rgb_camera.profile': '1280x720x30',
                'depth_module.profile': '1280x720x30',
                'pointcloud.enable': 'true'
            }.items()
    )

    return LaunchDescription([camera])