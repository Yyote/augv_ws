from setuptools import find_packages, setup

package_name = 'astar_planner'

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
    maintainer='leev',
    maintainer_email='yyootttaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar = astar_planner.adequate_astar:main',
            'astar_client = astar_planner.astar_client:main',
        ],
    },
    py_modules=[
        f'{package_name}.helper_functions',
        f'{package_name}.path_filter',
    ],
)
