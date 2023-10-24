from setuptools import find_packages, setup

package_name = 'mini_project'

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
    maintainer='n',
    maintainer_email='n.avontuur@outlook.com',
    description='Your package description',  # Replace with an actual description
    license='Apache License 2.0',  # Replace with the appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'objectdetection_sender_node = mini_project.object_detection_publisher:main',
            'objectdetection_reciever_node = mini_project.object_detection_subscriber:main',
            'movement_node = mini_project.Movement_node:main',
            'planner_node = mini_project.planner_node:main',
        ],
    },
)
