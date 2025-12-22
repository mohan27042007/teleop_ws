from setuptools import find_packages, setup

package_name = 'rover_path_planner'

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
    maintainer='mohanarangan-t-r',
    maintainer_email='mail4mohan27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'astar_planner = rover_path_planner.astar_planner:main',
        'rpp_controller = rover_path_planner.rpp_controller:main',
        'planner_manager = rover_path_planner.planner_manager:main',        

],
    },
)
