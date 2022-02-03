from setuptools import setup

package_name = 'carebt_navigator'

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
    description='CareBT navigator demo.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carebt_navigator = carebt_navigator.bt_navigator:main',
        ],
    },
)