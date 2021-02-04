from setuptools import setup
import os
from glob import glob 

package_name = 'sdp_launch'

data_files = []

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append((os.path.join('share', package_name), glob('launch/*.launch.py')))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='kpija',
    maintainer_email='kpijanowski99@gmail.com',
    description='SDP Simulation Launcher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
