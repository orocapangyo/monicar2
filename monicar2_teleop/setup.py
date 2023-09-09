import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'monicar2_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Bishop Pearson',
    author_email='bishopearson@gmail.com',
    maintainer='Bishop Pearson',
    maintainer_email='bishopearson@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard or joystick for Monicar2'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = monicar2_teleop.script.teleop_keyboard:main',
            'teleop_joy = monicar2_teleop.script.teleop_joy:main' 
        ],
    },
)
