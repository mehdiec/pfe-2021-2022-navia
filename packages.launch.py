import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():

    config_driver_anafi = os.path.join(os.getcwd(),'anafi','config','params.yaml')
    
    config_controller = os.path.join(os.getcwd(),'anaficontroller','config','params.yaml')

    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    
    config_camera = os.path.join(os.getcwd(),'camerasetup','config','params.yaml')
    
    config_manageimage = os.path.join(os.getcwd(),'manageimage','config','params.yaml')
    
    config_driver_viz = os.path.join(os.getcwd(),'visualisation','config','params.yaml')

    return launch.LaunchDescription([
    
        ### Anafi package
    
    	launch_ros.actions.Node(
            package='anafi',
            executable='anafi_driver',
            name='anafi_driver',
            parameters = [config_driver_anafi]
        ),
        launch_ros.actions.Node(
            package='anafi',
            executable='anafi_broadcaster',
            name='anafi_broadcaster'
        ),
        
        ### Anaficontroller package
        
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath]),
            
             
        launch_ros.actions.Node(
            package='anaficontroller',
            executable='controller',
            name='controller_node',
            parameters = [config_controller]
        ),
        
        ## Camera setup package   
        launch_ros.actions.Node(
            package='camerasetup',
            executable='camerainfo',
            name='camerainfo',
            parameters = [config_camera]
        ),
        
        ## Manageimage package
        launch_ros.actions.Node(
            package='manageimage',
            executable='image_segmentation',
            name='image_segmentation',
            parameters = [config_manageimage],
            remappings=[
                ("initial_image",  "anafi/image_raw"),
                ("segmented_image", "anafi/segmented_image")]
        ),
        
        ### Translation broadcaster
        launch_ros.actions.Node(
            package='translation_broadcaster',
            executable='broadcast',
            name='broadcast',
         
            ),
            
        #### 
        
        ### Visualisation package
            
        launch_ros.actions.Node(
            package='visualisation',
            executable='drone_viz',
            name='drone_viz',
            output = 'screen',
            #parameters = [config_driver_anafi]
            parameters = [{"focale" : 78e-3,}],
            remappings=[
                ("image_in",  "/anafi/segmented_image"),
                ("output_topic", "sensor_msgs/PointCloud2")]
        ),
        launch_ros.actions.Node(
            package='visualisation',
            executable='trajectory_viz',
            name='trajectory_viz',
            #output = 'screen',
            #parameters = [config_driver_viz]
            
        ),
        
        ### rviz
        
        launch_ros.actions.Node(
            package='rviz2', executable='rviz2', name='rviz2',
           ),

        ### video package node
        launch_ros.actions.Node(
            package='manageimage', executable='video_publisher_test', name='video_publisher_test',
           )
    ])
    
if __name__=="__main__":
    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
