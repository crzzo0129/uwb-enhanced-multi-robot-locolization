from setuptools import setup

package_name = 'multi_robot_trilat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trilat.launch.py']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='rhw',
    maintainer_email='renhaowen77@outlook.com',
    description='Trilateration fusion using odom and imu to estimate rb1 pose in rb2 frame',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tri_node_rb2base = multi_robot_trilat.tri_node_rb2base:main',
            'tri_node_rb1base = multi_robot_trilat.tri_node_rb1base:main',
            'tri_node_rb3base = multi_robot_trilat.tri_node_rb3base:main'
        ],
    },
)
