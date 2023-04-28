import os
from glob import glob
from setuptools import setup

package_name = 'sofar_crane_simulator'
lib = 'sofar_crane_simulator/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone.maccio@edu.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_sim_node = sofar_crane_simulator.crane_sim_node:main',
            'motor_x_node = sofar_crane_simulator.motor_x_controller:main',
            'motor_y_node = sofar_crane_simulator.motor_y_controller:main',
            'robot_logic_node = sofar_crane_simulator.robot_logic:main'
        ],
    },
)
