from setuptools import setup

package_name = 'odom_noise'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom_noise.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhw',
    maintainer_email='rhw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_noise_node = odom_noise.odom_noise_node:main',
        ],
    },
)
