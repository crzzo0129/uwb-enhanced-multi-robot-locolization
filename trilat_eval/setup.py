from setuptools import setup

package_name = 'trilat_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/eval_polyline.launch.py']),
        ('share/' + package_name + '/launch', ['launch/eval_circle.launch.py']),
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
            'eval_circle_node = trilat_eval.eval_circle_node:main',
            'eval_polyline_node = trilat_eval.eval_polyline_node:main',
        ],
    },
)
