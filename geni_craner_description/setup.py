import os
from glob import glob
from setuptools import setup

package_name = 'geni_craner_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),  # ← 改这里
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='URDF description package for geni_craner',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
