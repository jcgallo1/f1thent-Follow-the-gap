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
            'raceline_follower = reactive_race.raceline_follower:main',  
            'raceline_obs = reactive_race.raceline_obs:main', 
         
        ], 
    },
)
