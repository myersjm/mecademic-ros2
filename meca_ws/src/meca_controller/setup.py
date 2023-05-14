from setuptools import setup

package_name = 'meca_controller'

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
    maintainer='jessicamyers',
    maintainer_email='myersjm@rose-hulman.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ # package name, file name, func
        'console_scripts': [
            "meca_driver = meca_controller.meca_driver:main",
            "meca_driver_test = meca_controller.meca_driver_test:main",
            "meca_control = meca_controller.meca_control:main"
        ],
    },
)
