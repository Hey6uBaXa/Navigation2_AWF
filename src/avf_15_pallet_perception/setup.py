from setuptools import find_packages, setup

package_name = 'avf_15_pallet_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avf15',
    maintainer_email='you@example.com',
    description='Markerless pallet opening detector for AVF-15 fork docking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pallet_detector_node = avf_15_pallet_perception.pallet_detector_node:main',
        ],
    },
)
