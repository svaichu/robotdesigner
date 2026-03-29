
from pathlib import Path

header_import = """
from setuptools import find_packages, setup
import os
from glob import glob
"""

package_name = """
package_name = '{0}'
"""
setup_func = """
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
"""

def writeSetup(robot, setup_path):
    with open(setup_path, "w+") as f:
        f.write(header_import)
        f.write(package_name.format(robot.name))
        f.write(setup_func)
