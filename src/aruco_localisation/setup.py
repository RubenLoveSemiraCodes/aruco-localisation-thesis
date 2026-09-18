from setuptools import find_packages, setup

package_name = 'aruco_localisation'

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
    maintainer='lovesemira',
    maintainer_email='n11281316@qut.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = aruco_localisation.detector:main',
            'tf_publisher = aruco_localisation.tf_publisher:main',
            'optitrack_transform = aruco_localisation.optitrack_transform:main',
            'camera_rigidbody_extrinsic_calibrator = aruco_localisation.camera_rigidbody_extrinsic_calibrator:main',
        ],
    },
)
