from setuptools import setup

package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.2.1',
    packages=[],
    py_modules=[
        'nodes.axis'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Toffee Albina',
    author_email='toffee.albina@tri.global',
    maintainer='Toffee Albina',
    maintainer_email='toffee.albina@tri.global',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Axis camera using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis = nodes.axis:main',
        ],
    },
)
