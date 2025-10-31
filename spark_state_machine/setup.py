from setuptools import find_packages, setup

package_name = 'spark_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'scipy',
        'pyrealsense2',
        'rclpy',
        'std_msgs',
        'std_srvs',
    ],
    zip_safe=True,
    maintainer='divyansh',
    maintainer_email='2022ume1557@mnit.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wire_tip_detection = spark_state_machine.wire_tip_detection:main',
            'dh_gripper_node = spark_state_machine.dh_gripper_node:main',
            'gripper_stress_tester = spark_state_machine.gripper_stress_tester:main'
        ],
    },
)
