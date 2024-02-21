import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'monicar2_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Changwhan Lee',
    author_email='zeta0707@gmail.com',
    maintainer='Changwhan Lee',
    maintainer_email='zeta0707@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'monicar2 localization'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odomPublisher = monicar2_localization.script.odomPublisher:main',
            'rviz2ClickTo2d = monicar2_localization.script.rviz2ClickTo2d:main',
            'move = monicar2_localization.script.move:main',
            'rotate = monicar2_localization.script.rotate:main',
        ],
    },
)
