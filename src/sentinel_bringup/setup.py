from setuptools import setup

package_name = 'sentinel_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/video_and_yolo.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/video_rx.yaml',
            'config/image_preprocess.yaml',
            'config/bytetrack_tracker_eo.yaml',
            'config/bytetrack_tracker_ir.yaml',
            'config/yolo_detector_ir.yaml',
            'config/yolo_detector_eo.yaml',
            'config/ultralytics_yolo_eo.yaml',
            'config/ultralytics_yolo_eo_drone.yaml',
            'config/ultralytics_yolo_eo_person.yaml',
            'config/ultralytics_yolo_ir.yaml',
            'config/detection_merge_eo.yaml',
            'config/track_selector.yaml',
            'config/track_selector_eo.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup package for Sentinel system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
