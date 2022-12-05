from setuptools import setup
import os 
from glob import glob

package_name = 'manageimage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rudolf',
    maintainer_email='akiyo.worou@student-cs.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_segmentation = manageimage.image_segmentation:main',
            'publisher_test = manageimage.publisher_test:main',
            'subscriber_test = manageimage.subscriber_test:main',
            'video_publisher_test = manageimage.video_publisher_test:main'
        ],
    },
)
