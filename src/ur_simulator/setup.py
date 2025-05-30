from setuptools import find_packages, setup

package_name = 'ur_simulator'

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
    maintainer='fabio_tdt',
    maintainer_email='fabio_tdt@todo.todo',
    description='Fake UR publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_publisher = ur_simulator.ur_publisher:main',
        ],
    },
)
