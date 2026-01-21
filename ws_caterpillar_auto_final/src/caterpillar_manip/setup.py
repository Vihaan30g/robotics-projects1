from setuptools import find_packages, setup

package_name = 'caterpillar_manip'

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
    maintainer='satyajit',
    maintainer_email='satyajitgr9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_manip = caterpillar_manip.auto_manip:main',
            'limit_filter = caterpillar_manip.limit_filter:main',
        ],
        
    },
)
