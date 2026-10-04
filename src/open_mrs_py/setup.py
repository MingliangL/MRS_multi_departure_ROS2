import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'open_mrs_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mingliang',
    maintainer_email='liang@todo.todo',
    description='TODO: Package description',
    license='Apach-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_controller = open_mrs_py.vel_controller:main',
            'base_link_tf_pub = open_mrs_py.base_link_tf_pub:main'
        ],
    },
)
