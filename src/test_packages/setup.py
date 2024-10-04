from setuptools import setup

package_name = 'test_packages'

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
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Examples of minimal work ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = test_packages.my_node:main',
			'talker = test_packages.topic_publisher:main',
			'listener = test_packages.topic_subscriber:main',
			'server = test_packages.service_server:main',
			'client = test_packages.service_client:main',
            "action_server = test_packages.action_server:main",
            "action_client = test_packages.action_client:main"
        ],
    },
)
