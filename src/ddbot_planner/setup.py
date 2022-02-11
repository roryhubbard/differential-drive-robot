from setuptools import setup

package_name = 'ddbot_planner'

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
    maintainer='chub',
    maintainer_email='calvin3h@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_service = ddbot_planner.planner_service:main',
            'planner_client = ddbot_planner.planner_client:main',
        ],
    },
)
