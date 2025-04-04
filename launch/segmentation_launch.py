from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/us_image',
        description='Topic for the image input'
    )
    
    # Define the nodes
    bag_play_node = Node(
        package='scanlite_analysis_ros2',
        executable='rosbagtest.py',
        name='bag_play',
        output='screen'
    )

    transformation_node = Node(
        package='scanlite_analysis_ros2',
        executable='transformation.py',
        name='transformation',
        output='screen',
       )
    
    image_segmenter_node = Node(
        package='scanlite_analysis_ros2',
        executable='reconstruction_node.py',
        name='image_segmenter',
        output='screen',
        parameters=[{'image_topic': LaunchConfiguration('image_topic')}]
    )
    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(image_topic_arg)

    # Add the nodes
    ld.add_action(bag_play_node)
    ld.add_action(image_segmenter_node)
    ld.add_action(transformation_node)

    return ld 