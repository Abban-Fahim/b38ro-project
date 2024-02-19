from setuptools import find_packages, setup

package_name = 'joints'

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
    maintainer='Muhammad Abban',
    maintainer_email='lloydtechno58@gmail.com',
    description='Publishes joint angles to our robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker=joints.pub:main"
        ],
    },
)