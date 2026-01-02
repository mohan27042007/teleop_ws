from setuptools import find_packages, setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files:
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/description.launch.py',
            'launch/lidar.launch.py',
            'launch/teleop.launch.py',
            'launch/camera.launch.py',
            'launch/real_hardware.launch.py',
        ]),

        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
            'config/nav2_params_real.yaml',
            'config/twist_mux.yaml',
        ]),

        ('share/' + package_name + '/maps', [
            'maps/blank_map.yaml',
            'maps/blank_map.pgm',
        ]),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohanarangan-t-r',
    maintainer_email='mail4mohan27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
        ],
    },

)
