from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tacobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungho',
    maintainer_email='seungho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'home = tacobot.home:main',
            'scooper_grab = tacobot.scooper_grab:main',
            'scooper_pour = tacobot.pouring:main',
            'main_controller = tacobot.main_controller:main',
        ],
    },
)
