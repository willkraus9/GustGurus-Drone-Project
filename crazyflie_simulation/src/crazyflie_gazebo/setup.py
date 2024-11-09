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
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['models/**/*.sdf', 'models/**/*.config']},
    data_files=data_files, # The files to be installed
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Denis, Will, Kaustab, Nikolaj',
    maintainer_email='dkalpay@andrew.cmu.edu',

    description='Package for loading a crazyflie drone in gazebo. Also include a keyboard controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_services = crazyflie_gazebo.control_services:main',
        ],
    },
)
