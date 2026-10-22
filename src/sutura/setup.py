import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sutura'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SUTURA Team',
    maintainer_email='seu_email@exemplo.com',
    description='Suture robotisée autonome à faible coût',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_driver_node = sutura.drivers.arm_driver_node:main',
            'vision_node = sutura.vision.vision_node:main',
            'control_node = sutura.control.control_node:main',
            'planning_node = sutura.planning.planning_node:main'
        ],
    },
)