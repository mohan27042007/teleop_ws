from setuptools import find_packages, setup

package_name = 'rover_motor'

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
    description='rover motor bridge: cmd_vel -> motor pwm',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [
            'motor_node = rover_motor.motor_node:main'
        ],
    },
)
