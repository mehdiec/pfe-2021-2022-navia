import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_driver = os.path.join(
    get_package_share_directory('camerasetup'),'config','params.yaml')

    return launch.LaunchDescription([    
        launch_ros.actions.Node(
            package='camerasetup',
            executable='camerainfo',
            name='camerainfo',
            parameters = [config_driver]
        )
    ])
    