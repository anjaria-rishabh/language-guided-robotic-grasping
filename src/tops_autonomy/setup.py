from setuptools import setup
import os
from glob import glob

package_name = 'tops_autonomy'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                ('share/'+package_name,['package.xml']),
                (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
                (os.path.join('share',package_name,'config'),glob('config/*.yaml'))],
    install_requires=['setuptools'],zip_safe=True,
    maintainer='Rishabh Anjaria',maintainer_email='maintainer@example.invalid',
    description='ROS 2 language grounding, RGB-D localization, PBVS reference and contract bridge',
    license='Apache-2.0',
    entry_points={'console_scripts':[
        'perception_node = tops_autonomy.perception_node:main',
        'pbvs_node = tops_autonomy.pbvs_node:main',
        'ee_tf_node = tops_autonomy.ee_tf_node:main',
        'contract_bridge = tops_autonomy.contract_bridge:main',
    ]},
)
