from setuptools import find_packages, setup
import os

package_name = 'titration_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/titration_ui.launch.py']),
        ('share/' + package_name + '/ui', [os.path.join(package_name, 'ui', 'mainwindow.ui')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khkoh',
    maintainer_email='khkoh23@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'titration_ui = titration_ui.main:main'
        ],
    },
)
