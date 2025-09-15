from setuptools import find_packages, setup
from glob import glob

package_name = 'go2_driver_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', glob("rviz/*.rviz")),
        ('share/' + package_name + '/launch', glob("launch/*.launch.py")),
        ('share/' + package_name + '/params', glob("params/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nalishe',
    maintainer_email='nalishe9105@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = go2_driver_py.driver:main'
        ],
    },
)
