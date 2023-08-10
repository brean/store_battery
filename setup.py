from setuptools import find_packages, setup

package_name = 'store_battery'

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
    maintainer='abresser',
    maintainer_email='Andreas.Bresser@dfki.de',
    description='Empty the battery until we can store it',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lower_voltage = store_battery.lower_voltage:main'
        ],
    },
)
