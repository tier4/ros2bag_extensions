from setuptools import find_packages
from setuptools import setup

package_name = 'ros2bag_extensions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli', 'ros2bag'],
    zip_safe=True,
    author='Keisuke Shima',
    author_email='keisuke.shima@tier4.jp',
    maintainer='Keisuke Shima',
    maintainer_email='keisuke.shima@tier4.jp',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Extensions for rosbag in ROS 2',
    long_description="""\
The package provides the additional rosbag command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2bag.verb': [
            'filter = ros2bag_extensions.verb.filter:FilterVerb',
            'merge = ros2bag_extensions.verb.merge:MergeVerb',
            'slice = ros2bag_extensions.verb.slice:SliceVerb',
        ],
    }
)
