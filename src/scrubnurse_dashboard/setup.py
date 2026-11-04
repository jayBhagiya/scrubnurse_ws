import glob
import os

from setuptools import find_packages, setup

package_name = 'scrubnurse_dashboard'


def web_files():
    """Install the built web page (web/dist, from `npm run build`), if there is one."""
    files = [f for f in glob.glob('web/dist/**', recursive=True) if os.path.isfile(f)]
    share = os.path.join('share', package_name, 'web')
    return [(os.path.join(share, os.path.relpath(os.path.dirname(f), 'web/dist')), [f])
            for f in files]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *web_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jay Bhagiya',
    maintainer_email='jabh00002@stud.uni-saarland.de',
    description='Scrubnurse dashboard: watch and control the cell from a web page.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = scrubnurse_dashboard.dashboard_node:main',
            'metrics = scrubnurse_dashboard.metrics:main',
        ],
    },
)
