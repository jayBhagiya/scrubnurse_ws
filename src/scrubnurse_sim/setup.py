from glob import glob

from setuptools import find_packages, setup

package_name = 'scrubnurse_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jay Bhagiya',
    maintainer_email='jabh00002@stud.uni-saarland.de',
    description='Gazebo simulation of the Scrubnurse cell.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_gripper = scrubnurse_sim.sim_gripper_node:main',
            'sim_surgeon = scrubnurse_sim.sim_surgeon_node:main',
            'sim_tool_detector = scrubnurse_sim.sim_tool_detector_node:main',
        ],
    },
)
