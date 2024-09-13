from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_track = ugv_vision.color_track:main',
            'kcf_track = ugv_vision.kcf_track:main',
            'gesture_ctrl = ugv_vision.gesture_ctrl:main',
            'apriltag_ctrl = ugv_vision.apriltag_ctrl:main',
            'apriltag_track_0 = ugv_vision.apriltag_track_0:main',
            'apriltag_track_1 = ugv_vision.apriltag_track_1:main',
            'apriltag_track_2 = ugv_vision.apriltag_track_2:main'
        ],
    },
)
