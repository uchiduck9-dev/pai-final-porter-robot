import os
from glob import glob

from setuptools import setup

package_name = 'pai_final_bot'


def model_data_files():
    """models/ 以下をサブフォルダ（meshes, materials など）ごとインストール対象にする。"""
    files = []
    for root, _, names in os.walk('models'):
        if names:
            files.append((os.path.join('share', package_name, root),
                          [os.path.join(root, n) for n in names]))
    return files

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ] + model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai_final',
    maintainer_email='noreply@example.com',
    description='Differential-drive robot in Ignition Gazebo Fortress, driven from ROS 2 Humble.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'porter_node = pai_final_bot.porter_node:main',
        ],
    },
)
