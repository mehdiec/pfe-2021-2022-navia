import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_driver = os.path.join(
    get_package_share_directory('anafi'),'config','params.yaml')

    return launch.LaunchDescription([    
        launch_ros.actions.Node(
            package='anafi',
            executable='anafi_driver',
            name='anafi_driver',
            parameters = [config_driver]
        ),
        launch_ros.actions.Node(
            package='anafi',
            executable='anafi_broadcaster',
            name='anafi_broadcaster'
        )
        
    ])
    