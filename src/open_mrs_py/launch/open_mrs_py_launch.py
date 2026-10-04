import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   base_lin_tf = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('open_mrs_py'), 'launch'),
         '/base_link_tf_launch.py'])
      )
   controllers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('open_mrs_py'), 'launch'),
         '/controller_launch.py'])
      )

   return LaunchDescription([
      base_lin_tf,
      controllers
   ])
