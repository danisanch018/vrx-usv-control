import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'usv_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danisanch118',
    maintainer_email='dansancas@alum.us.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nmpc_control = usv_control_py.nmpc_control:main',
            'path_publisher = usv_control_py.path_publisher:main'
        ],
    },
)
