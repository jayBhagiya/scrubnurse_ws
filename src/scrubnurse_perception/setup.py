from glob import glob

from setuptools import find_packages, setup

package_name = 'scrubnurse_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jay Bhagiya',
    maintainer_email='jabh00002@stud.uni-saarland.de',
    description='Scrubnurse tool perception: YOLO tool segmentation and homography.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_detector = scrubnurse_perception.tool_detector_node:main',
            'tool_localizer = scrubnurse_perception.tool_localizer_node:main',
            'compute_homography = scrubnurse_perception.compute_homography:main',
            'collect_calibration = scrubnurse_perception.collect_calibration:main',
            'kinect_camera = scrubnurse_perception.kinect_camera_node:main',
        ],
    },
)
