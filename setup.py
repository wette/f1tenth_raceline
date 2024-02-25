from setuptools import find_packages, setup

package_name = 'raceline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/trajectory.py']),
        ('lib/' + package_name, [package_name+'/pid_controller.py']),
        ('lib/' + package_name, [package_name+'/pure_pursuit.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Philip Wette',
    maintainer_email='philip.wette@hsbi.de',
    description='Pure Pursuit raceline following car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = raceline.pure_pursuit:main',
        ],
    },
)