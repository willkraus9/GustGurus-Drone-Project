from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crazyflie_gazebo'

data_files=[
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),   
    ]

for file_path in glob('models/**/*', recursive=True):
    if os.path.isfile(file_path):  # Only add files, skip directories
        target_path = os.path.join('share', package_name, file_path)
        data_files.append((os.path.dirname(target_path), [file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files, # The files to be installed
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='denis',
    maintainer_email='denis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_services = crazyflie_gazebo.control_services:main'
        ],
    },
)
