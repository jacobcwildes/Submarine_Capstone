from setuptools import setup
import os

package_name = 'submarine_coms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['../../launch/sub_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Dunton',
    maintainer_email='dyllon.dunton@maine.edu',
    description='commands from controller exec and data report',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_coms = submarine_coms.sub_coms:main'
        ],
    },
)
