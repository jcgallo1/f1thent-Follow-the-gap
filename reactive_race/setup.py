from setuptools import find_packages, setup

package_name = 'reactive_race'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juangallo',
    maintainer_email='juangallo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactive_node = reactive_race.reactive_node:main',
            'waypoint_recorder = reactive_race.waypoint_recorder:main', 
            'manual_driver = reactive_race.manual_driver:main',
            'raceline_follower = reactive_race.raceline_follower:main',  
         
        ], 
    },
)
