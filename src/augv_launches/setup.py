from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'augv_launches'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # package_dir={'': 'launch'},
    # py_modules=[
    #     "potential_fields_params",
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*_params.py')),
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
        ],
    },
)
