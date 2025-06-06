from setuptools import find_packages, setup

package_name = 'slambot_odometry_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/slambot_odometry_pkg']),
        ('share/slambot_odometry_pkg', ['package.xml']),
        ('share/slambot_odometry_pkg/launch', ['launch/slambot_odometry.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter',
    maintainer_email='peter.wallhead@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odometry_publisher = slambot_odometry_pkg.odometry_publisher:main",
        ],
    },
)
