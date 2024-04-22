from setuptools import find_packages, setup
import glob
import os 

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ito',
    maintainer_email='ito@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'img_publisher = camera_package.img_publisher:main',
             'canny = camera_package.canny:main',
             'canny_with_cam = camera_package.canny_with_cam:main',
             'camera_server = camera_package.camera_server:main',
             'video_server = camera_package.video_server:main',
             'video_client = camera_package.video_client:main'
        ],
    },
)
