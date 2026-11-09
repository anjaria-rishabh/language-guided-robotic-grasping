import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tops_experiments'

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
    description='Trial logging, the batch runner, and the statistics pipeline.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trial_logger = tops_experiments.trial_logger:main',
            'trial_control = tops_experiments.trial_control:main',
            'run_trials = tops_experiments.run_trials:main',
            'analyze_trials = tops_experiments.analyze_trials:main',
            'plot_trials = tops_experiments.plot_trials:main',
        ],
    },
)
