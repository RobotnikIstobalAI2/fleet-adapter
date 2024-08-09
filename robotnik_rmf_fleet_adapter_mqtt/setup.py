from setuptools import setup

package_name = 'robotnik_rmf_fleet_adapter_mqtt'

setup(
    name=package_name,
    version='0.2.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emima Jiva',
    maintainer_email='emji@ai2.upv.es',
    description='Robotnik MQTT RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=robotnik_rmf_fleet_adapter_mqtt.fleet_adapter:main',
            'dispatch_action=robotnik_rmf_fleet_adapter_mqtt.dispatch_action:main'
        ],
    },
)
