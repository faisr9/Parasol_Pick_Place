from setuptools import find_packages, setup

package_name = 'ur5e_director'

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
    maintainer='Reid Faistl',
    maintainer_email='rfaistl2@illinois.edu',
    description='Coordinates with moveit2 to execute motions on the UR5e arm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'director_node = ur5e_director.director_node:main',
            'talker = ur5e_director.publisher_member_function:main',
            'listener = ur5e_director.subscriber_member_function:main',
        ],
    },
)
