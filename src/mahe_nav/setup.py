from setuptools import setup
import os
from glob import glob

package_name = 'mahe_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=['mahe_nav'],   # ✅ FORCE include
    py_modules=[],           # ✅ avoid conflicts
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahe',
    maintainer_email='mahe@todo.todo',
    description='Navigation nodes for MAHE UGV autonomous challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector = mahe_nav.aruco_detector_node:main',
            'sign_detector = mahe_nav.sign_detector_node:main',
            'lidar_analyzer = mahe_nav.lidar_analyzer_node:main',
            'nav_controller = mahe_nav.nav_controller_node:main',
            'status_logger = mahe_nav.status_logger_node:main',
        ],
    },
)
