import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tops_supervisor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kabir',
    maintainer_email='kartikeyharshkabir@gmail.com',
    description='Grasp sequencing FSM and safety interlocks for the TOPS stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'supervisor_node = tops_supervisor.supervisor_node:main',
        ],
    },
)
