from setuptools import find_packages, setup

package_name = 'berry_dataset'

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
    description='Save blackberry data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "berry_dataset = berry_dataset.berry_dataset:main",
            "dataset_saver = berry_dataset.dataset_saver:main"
        ],
    },
)
