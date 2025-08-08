from setuptools import setup

package_name = 'platform_lander'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Platform detection, velocity alignment, and autonomous landing node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_tracker = platform_lander.vision_tracker:main',
            'align_controller = platform_lander.align_controller:main',
        ],
    },
)

