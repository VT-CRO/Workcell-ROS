from setuptools import find_packages, setup

package_name = 'mc2425'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='aviola@vt.edu',
    description='Manufacturing Workcell ROS Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'addPrint = mc2425.addPrint:main',
            'mainController = mc2425.mainController:main',
            'shelfUpdate = mc2425.shelfUpdate:main',
            'pickAndPlace = mc2425.pickAndPlace:main',
        ],
    },
)