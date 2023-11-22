from setuptools import setup
import os

package_name = 'controller_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['../../launch/con_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob wildes',
    maintainer_email='wildes126@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'con_gui = controller_gui.con_gui:main'
        ],
    },
)
