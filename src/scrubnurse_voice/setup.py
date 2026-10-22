from setuptools import find_packages, setup

package_name = 'scrubnurse_voice'

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
    description='Scrubnurse voice commands: Whisper speech recognition to tool ids.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command = scrubnurse_voice.voice_command_node:main',
        ],
    },
)
