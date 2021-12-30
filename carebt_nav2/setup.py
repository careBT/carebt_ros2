from setuptools import setup

package_name = 'carebt_nav2'

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
    maintainer='Andreas Steck',
    maintainer_email='steck.andi@gmail.com',
    description='CareBT nodes for the Navigation2 stack.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_helloWorld = carebt_nav2.bt_helloWorld:main',
            'bt_computePathToPose = carebt_nav2.bt_computePathToPose:main',
        ],
    },
)
