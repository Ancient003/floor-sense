from glob import glob

from setuptools import find_packages, setup

package_name = 'floor_sense'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*.html', 'web/fonts/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ancient003',
    maintainer_email='ancient003@todo.todo',
    description='Serial bridge between the Floor Sense STM32 module and ROS 2.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'bridge = floor_sense.bridge_node:main',
            'fake_nucleo = floor_sense.fake_nucleo:main',
            'dashboard = floor_sense.dashboard_node:main',
        ],
    },
)
