from setuptools import find_packages, setup

package_name = 'tops_perception_stub'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kabir',
    maintainer_email='kartikeyharshkabir@gmail.com',
    description='Ground-truth grasp target publisher with seeded disturbance regimes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grasp_publisher = tops_perception_stub.grasp_publisher:main',
        ],
    },
)
