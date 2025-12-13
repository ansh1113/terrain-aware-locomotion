from setuptools import setup
import os
from glob import glob

package_name = 'terrain_locomotion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        f'{package_name}.perception',
        f'{package_name}.perception.models',
        f'{package_name}.planning',
        f'{package_name}.control'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ansh Bhansali',
    maintainer_email='anshbhansali5@gmail.com',
    description='Terrain-aware locomotion pipeline for quadruped robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_classifier = terrain_locomotion.perception.terrain_classifier:main',
            'elevation_mapper = terrain_locomotion.perception.elevation_mapper:main',
            'footstep_planner = terrain_locomotion.planning.footstep_planner:main',
            'gait_controller = terrain_locomotion.control.gait_controller:main',
            'safety_monitor = terrain_locomotion.control.safety_monitor:main',
            'demo_pipeline = terrain_locomotion.demo_pipeline:main',
        ],
    },
)