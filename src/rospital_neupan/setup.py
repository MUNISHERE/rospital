from setuptools import setup

package_name = 'rospital_neupan'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rospital_neupan_launch.py']),
        ('share/' + package_name + '/config', ['rospital_neupan/config/neupan_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 NeuPAN planner for Rospital robot',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'neupan_node = rospital_neupan.neupan_node:main',
        ],
    },
)
