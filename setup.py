from setuptools import setup
import os
from glob import glob

package_name = 'controller_pkg'
controller_submodule = str(package_name + "/controller_submodule")
state_estimate_submodule = str(package_name + "/state_estimate_submodule")

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, controller_submodule, state_estimate_submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'controller_submodule'), glob('controller_submodule/*.py')),
        (os.path.join('share', package_name, 'state_estimate_submodule'),glob('state_estimate_submodule/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_node = controller_pkg.lqr_node:main',
            'lqg_node = controller_pkg.lqg_node:main',
            'mpc_node = controller_pkg.mpc_node:main',
            'pid_node = controller_pkg.pid_node:main'
        ],
    },
)
