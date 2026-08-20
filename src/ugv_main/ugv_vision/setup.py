from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'ugv_vision': [
            'models/oak/*.blob',
            'models/ultraface-ncnn/*',
        ],
    },
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
            'cam_webrtc = ugv_vision.cam_webrtc:main',
            'color_select = ugv_vision.color_select:main',
            'pt_color_ball_track = ugv_vision.pt_color_ball_track:main',
            'pt_apriltag_track = ugv_vision.pt_apriltag_track:main',
            'pt_face_track = ugv_vision.pt_face_track:main',
            'pt_gesture_ctrl = ugv_vision.pt_gesture_ctrl:main',
            'color_ball_track = ugv_vision.color_ball_track:main',
            'apriltag_track = ugv_vision.apriltag_track:main',
            'apriltag_track_nav2 = ugv_vision.apriltag_track_nav2:main',
            'color_line_follow = ugv_vision.color_line_follow:main',
            'roarm_color_line_follow = ugv_vision.roarm_color_line_follow:main',
            'face_track = ugv_vision.face_track:main',
            'gesture_ctrl = ugv_vision.gesture_ctrl:main',
            'cam_oak_webrtc = ugv_vision.cam_oak_webrtc:main',
            'oak_color_select = ugv_vision.oak_color_select:main',
            'oak_color_ball_track = ugv_vision.oak_color_ball_track:main',
            'oak_object_track = ugv_vision.oak_object_track:main',
        ],
    },
)
