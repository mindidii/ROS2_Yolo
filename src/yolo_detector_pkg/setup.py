from setuptools import setup
from glob import glob

package_name = 'yolo_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    scripts=[
        'scripts/object_detector_node',
        'scripts/yolo_detector_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Minimal image subscriber node for ROS2',
    license='TODO',
    tests_require=['pytest'],
)
