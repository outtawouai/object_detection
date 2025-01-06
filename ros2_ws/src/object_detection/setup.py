from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='outtawouai@github.com',
    description='Sample object detection node for pedestrians',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_node = object_detection.video_node:main',
            'detector_node = object_detection.object_detection_node:main',
            'pose_detector_node = object_detection.pose_detection_node:main',
            'display_node = object_detection.display_node:main',
        ],
    },
)
