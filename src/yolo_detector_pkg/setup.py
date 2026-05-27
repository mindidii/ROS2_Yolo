from setuptools import find_packages
from setuptools import setup
from pathlib import Path

package_name = 'yolo_detector_pkg'

package_dir = Path(__file__).parent
model_files = [str(path) for path in (package_dir / 'model').glob('*') if path.is_file()]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    scripts=[
        'scripts/bytetrack_tracker_node',
        'scripts/debug_detection_viewer',
        'scripts/detection_merge_node',
        'scripts/track_selector_node',
        'scripts/ultralytics_yolo_node',
        'scripts/web_detection_viewer',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Minimal image subscriber node for ROS2',
    license='TODO',
    tests_require=['pytest'],
)
