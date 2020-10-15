from setuptools import setup
import os
from glob import glob

# File not used. Change the build tool in package.xml to change the behaviour

package_name = 'uuv_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    #scripts=['scripts/finned_uuv_teleop.py', 'scripts/vehicle_keyboard_teleop.py', 'scripts/vehicle_teleop.py']
    entry_points={
        'console_scripts': [
            'finned_uuv_teleop = scripts.finned_uuv_teleop:main',
            'vehicle_keyboard_teleop = scripts.vehicle_keyboard_teleop:main',
            'vehicle_teleop = scripts.vehicle_teleop:main'
        ],
    },
)
