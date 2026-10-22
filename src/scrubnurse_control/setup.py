from setuptools import find_packages, setup

package_name = 'scrubnurse_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jay Bhagiya',
    maintainer_email='jabh00002@stud.uni-saarland.de',
    description='Scrubnurse task controller: MoveIt 2 pick-and-place of surgical tools.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scrubnurse_controller = scrubnurse_control.controller_node:main',
            'record_pose = scrubnurse_control.record_pose:main',
        ],
    },
)
