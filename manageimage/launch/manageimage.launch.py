import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_driver = os.path.join(
    get_package_share_directory('manageimage'),'config','params.yaml')

    return launch.LaunchDescription([    
        launch_ros.actions.Node(
            package='manageimage',
            executable='image_segmentation',
            name='image_segmentation',
            parameters = [config_driver],
            remappings=[
                ("initial_image",  "anafi/image_raw"),
                ("segmented_image", "anafi/segmented_image")]
        )
    ])
    