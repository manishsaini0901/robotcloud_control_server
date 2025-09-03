from setuptools import setup

package_name = 'robotcloud_control_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    package_data={
        '': ['package.xml'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robotcloud_control_server']),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='manish',
    maintainer_email='manish@example.com',
    description='Robot control services using robotcloud_msgs',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_state_server = robotcloud_control_server.robot_state_server:main',
            'task_planner_server = robotcloud_control_server.task_planner_server:main',
        ],
    },
)
