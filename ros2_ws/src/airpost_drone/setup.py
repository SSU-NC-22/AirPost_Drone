from setuptools import setup
package_name = 'airpost_drone'
setup(
    name=package_name, version='1.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', ['launch/all_drone_control.launch.py'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='SSU NC-Lab', description='AirPost drone controller (ROS2/PX4 v1.17)',
    license='MIT',
    entry_points={'console_scripts': [
        'drone_node = airpost_drone.drone_node:main',
        'dummy_camera = airpost_drone.dummy_camera:main',
    ]},
)
