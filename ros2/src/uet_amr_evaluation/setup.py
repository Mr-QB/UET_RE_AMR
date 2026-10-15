from setuptools import find_packages, setup

package_name = 'uet_amr_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Department of Robotics',
    maintainer_email='robotics@uet.vnu.edu.vn',
    description='Odometry accuracy tests (straight / rotation) with CSV logging and report plots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_test = uet_amr_evaluation.odom_test:main',
            'odom_report = uet_amr_evaluation.odom_report:main',
        ],
    },
)
