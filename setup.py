import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_control_VBA'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include Gazebo worlds (assuming they are located in a 'worlds' directory)
        #(os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        # Add Gazebo launch files if applicable
        #(os.path.join('share', package_name, 'gazebo'), glob(os.path.join('gazebo', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vinicius',
    maintainer_email='viniciusbaltoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_VBA.turtle_control:main'
        ],
    },
)
