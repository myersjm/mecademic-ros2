from setuptools import setup

package_name = 'meca_motion_planner'

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
    entry_points={
        'console_scripts': [
            "motion_planner = meca_motion_planner.motion_planner:main",
            "test_msg_srv_service = meca_motion_planner.test_msg_srv_service:main",
            "test_msg_srv_client = meca_motion_planner.test_msg_srv_client:main",
            "test_motion_planner= meca_motion_planner.test_motion_planner:main"
        ],
    },
)
