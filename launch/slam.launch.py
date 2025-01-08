import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # YDLIDAR X4 ドライバの起動
        DeclareLaunchArgument(
            'frame_id', default_value='laser_link', description='Frame ID for laser scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([LaunchConfiguration('package_share_directory'), '/launch/ydlidar_launch.py']),
        ),

        # SLAMツールボックスの起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([LaunchConfiguration('package_share_directory'), '/launch/slam_launch.py']),
        ),

        # RVizの起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([LaunchConfiguration('package_share_directory'), '/launch/rviz_launch.py']),
        ),

        # SLAMツールボックスの設定（もし必要なら）
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # 実際の時間を使用する場合
                'slam_params_file': '/path/to/your/slam_toolbox_params.yaml',  # SLAMパラメータ
            }],
            remappings=[('/scan', '/laser_scan')]  # LIDARのスキャンデータのトピックを指定
        ),

        # RVizでの可視化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', '/path/to/your/rviz_config.rviz'],  # RVizの設定ファイルを指定
        ),
    ])
