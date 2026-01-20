from setuptools import setup
import os
from glob import glob

package_name = 'swarm'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # Robot model files
        ('share/' + package_name + '/model', glob('model/*')),

        # Meshes
        ('share/' + package_name + '/meshes', glob('meshes/*')),

        # World files
        ('share/' + package_name + '/world', glob('world/*.sdf')),

        # Subfolders inside world/
        ('share/' + package_name + '/world/ground', package_files('world/ground')),
        # Parameters
        ('share/' + package_name + '/parameters', ['parameters/bridge_params.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Animesh Mishra',
    maintainer_email='animeshmishra211@gmail.com',
    description='A Package for learning ROS and Gazebo by implementing it in Swarm Robotics applications',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
             'bot_teleop = swarm.bot_teleop:main',
        ],
    },
)
