from setuptools import setup

package_name = 'cxw_carto'

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
    maintainer='cxw',
    maintainer_email='cxw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'receive_laser = cxw_carto.receive_laser:main',
        'receive_imu = cxw_carto.receive_imu:main',
        'receive_odom = cxw_carto.receive_odom:main',
        'receive_data = cxw_carto.receive_data:main',
        'myMap = cxw_carto.myMap:main',
        'test = cxw_carto.test:main',
        'scanMatch = cxw_carto.scanMatch:main',
        'myBoardcaster = cxw_carto.myBoardcaster:main'
        'test = cxw_carto.test:main'
        ],
    },
)
