from setuptools import find_packages, setup

package_name = 'realsense_streamer'

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
    description='Stream Realsense Camera',
    license='pache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RealSenseStreamer = realsense_streamer.RealSenseStreamer:main',
        ],
    },
)
