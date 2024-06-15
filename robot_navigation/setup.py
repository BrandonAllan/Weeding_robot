from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('launch/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brandon',
    maintainer_email='allanbrandon34@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_detector_yolov5 = robot_navigation.crop_detector_yolov5:main',
            'crop_detector_yolov8 = robot_navigation.crop_detector_yolov8:main',
            'crop_follower = robot_navigation.crop_follower:main',
            'crop_turning = robot_navigation.crop_turning:main',
            #'navigation_node = robot_navigation.navigation_node:main',
        ],
    },
)
