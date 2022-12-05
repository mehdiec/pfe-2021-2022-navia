import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_driver = os.path.join(
    get_package_share_directory('visualisation'),'config','params.yaml')

    return launch.LaunchDescription([    
        launch_ros.actions.Node(
            package='visualisation',
            executable='drone_viz',
            name='drone_viz',
            #output = 'screen',
            #parameters = [config_driver]
            #parameters = [{"focale" : 78e-3,}],
            remappings=[
                ("image_in",  "/anafi/segmented_image"),
                ("output_topic", "sensor_msgs/PointCloud2")]
        ),
        launch_ros.actions.Node(
            package='visualisation',
            executable='trajectory_viz',
            name='trajectory_viz',
            #output = 'screen',
            #parameters = [config_driver]
            
        )
    ])
    
