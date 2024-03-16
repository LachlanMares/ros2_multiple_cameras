from setuptools import setup

package_name = 'multiple_cameras'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlanmares',
    maintainer_email='lachlan.mares@adelaide.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiple_camera_publisher = multiple_cameras.multiple_camera_publisher_python:main',
            'multiple_camera_publisher = multiple_cameras.compressed_to_image_publisher_python:main',
        ],
    },
)
