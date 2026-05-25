from setuptools import find_packages, setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Denis Alpay',
    maintainer_email='dkalpay@andrew.cmu.edu',
    description='Python controller nodes for the GustGurus Crazyflie simulation: publishes per-motor velocity commands to the drone in Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MotorControlNode = controllers.MotorControlNode:main',
        ],
    },
)
