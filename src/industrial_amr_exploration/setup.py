from glob import glob

from setuptools import find_packages, setup

package_name = 'industrial_amr_exploration'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Achref Tebini',
    maintainer_email='tebiniachref@gmail.com',
    description='Frontier exploration and Nav2 tooling for the industrial AMR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = industrial_amr_exploration.frontier_explorer:main',
            'waypoint_runner = industrial_amr_exploration.waypoint_runner:main',
            'global_localizer = industrial_amr_exploration.global_localizer:main',
        ],
    },
)
