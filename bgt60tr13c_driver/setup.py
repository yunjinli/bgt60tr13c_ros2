from setuptools import find_packages, setup
from glob import glob 
package_name = 'bgt60tr13c_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/radar.yaml']),
        ('share/' + package_name + '/config', ['config/qrsd.onnx']),
        ('share/' + package_name + '/config', ['config/rsd.pth']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunjinli',
    maintainer_email='yunjin.li0817@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_node = bgt60tr13c_driver.radar_node:main',
            'raw_frame_viz = bgt60tr13c_driver.raw_frame_viz:main',
            'raw_frame_viz_cmap = bgt60tr13c_driver.raw_frame_viz_cmap:main',
            'radar_surface_detection_node = bgt60tr13c_driver.radar_surface_detection_node:main',
            'radar_surface_detection_onnx_node = bgt60tr13c_driver.radar_surface_detection_onnx_node:main',
        ],
    },
)
