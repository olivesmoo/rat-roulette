from setuptools import find_packages, setup

package_name = 'rat_roulette_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/roulette_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtwig',
    maintainer_email='om228@cornell.edu',
    description='This package implements an interactive simplified roulette game using a hybrid robot control architecture.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rat_roulette_node = rat_roulette_pkg.rat_roulette_node:main'
        ],
    },
)
