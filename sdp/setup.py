from setuptools import setup
import os
from glob import glob 

package_name = 'sdp'

data_files = []

data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)

data_files.append(
    (os.path.join('share', package_name, 'protos'), ['protos/ClayUno.proto'])
)

data_files.append(
    (os.path.join('share', package_name, 'worlds'), ['worlds/tennis_court.wbt'])
)

data_files.append(
    (os.path.join('share', package_name, 'worlds', 'textures'), glob('worlds/textures/*'))
)

data_files.append(
    ('share/' + package_name, ['package.xml'])
)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpija',
    maintainer_email='kpijanowski99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = sdp.file:main',
            'listener = sdp.file2:main',
            'start_robot = sdp.slave_controller:main',
            'aruco_navigator = sdp.aruco_navigator:main',
        ],
    },
)
