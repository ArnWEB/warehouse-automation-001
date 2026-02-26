from setuptools import find_packages, setup

package_name = 'cuopt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cuopt_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xelf',
    maintainer_email='xelf@todo.todo',
    description='CuOpt Bridge for warehouse automation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cuopt_bridge = cuopt_bridge.cuopt_bridge:main',
            'cuopt_client = cuopt_bridge.cuopt_client:main',
        ],
    },
)
