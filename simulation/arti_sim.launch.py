from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
 root=Path(__file__).resolve().parent; project=root.parent
 def inc(pkg,file,args):
  return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(pkg))/'launch'/file)),launch_arguments=args.items())
 common={'namespace':'robot4','use_sim_time':'true'}
 resources=':'.join([str(root),str(Path(share('turtlebot4_description')).parent),str(Path(share('irobot_create_description')).parent),str(Path(share('irobot_create_gz_bringup'))/'worlds')])
 return LaunchDescription([
  DeclareLaunchArgument('x',default_value='-0.2'),DeclareLaunchArgument('y',default_value='0.0'),DeclareLaunchArgument('yaw',default_value='0.0'),
  DeclareLaunchArgument('skip_undock',default_value='false'),
  DeclareLaunchArgument('simulate_docking',default_value='false'),
  SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),
  inc('ros_gz_sim','gz_sim.launch.py',{'gz_args':f'{root}/arti_gallery.sdf -r -v 2'}),
  Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
  inc('turtlebot4_gz_bringup','turtlebot4_spawn.launch.py',{**common,'world':'arti_gallery','model':'lite','rviz':'false','slam':'false','nav2':'false','localization':'false','x':LaunchConfiguration('x'),'y':LaunchConfiguration('y'),'yaw':LaunchConfiguration('yaw')}),
  inc('turtlebot4_navigation','localization.launch.py',{**common,'map':str(project/'maps/gallery.yaml'),'params':str(root/'localization.yaml')}),
  inc('turtlebot4_navigation','nav2.launch.py',{**common,'params_file':str(root/'nav2_full_sim.yaml')}),
  Node(package='rviz2',executable='rviz2',namespace='robot4',additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},arguments=['-d',str(root/'arti.rviz')],parameters=[{'use_sim_time':True}],remappings=[('/tf','tf'),('/tf_static','tf_static')]),
  # PC2 course controller: start only when /robot4/arti/tour/select is received.
  # IMPORTANT: fill the surveyed positions in arti_pc2_nav/config/a_course.yaml first.
  Node(package='arti_pc2_nav', executable='a_course_node', name='a_course_node',
       namespace='robot4', output='screen', parameters=[{
         'use_sim_time':True, 'config_file':str(Path(share('arti_pc2_nav'))/'config/a_course.yaml'),
         'simulation_mode':True,
         'skip_undock':ParameterValue(LaunchConfiguration('skip_undock'), value_type=bool),
         'simulate_docking':ParameterValue(LaunchConfiguration('simulate_docking'), value_type=bool),
         'require_dock_status':True, 'enable_motion':True,
         'simulation_battery_percent':75.0, 'require_tf':True}],
       remappings=[('/tf','tf'),('/tf_static','tf_static')]),
  Node(package='arti_pc2_nav', executable='zone_marker_node', name='arti_zone_markers',
       namespace='robot4', output='screen', parameters=[{
         'use_sim_time':True, 'config_file':str(Path(share('arti_pc2_nav'))/'config/a_course.yaml'),
         'map_frame':'map'}],
       remappings=[('/tf','tf'),('/tf_static','tf_static')]),
 ])
