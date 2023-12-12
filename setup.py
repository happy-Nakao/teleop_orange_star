import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='c1203567@st.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_teleop = teleop.base_teleop:main',
            'base_test = teleop.base_test:main',
            'base_joycon = teleop.base_joycon:main',
            'base_keyboard = teleop.base_keyboard:main',
            'base_keyboard_gpt = teleop.base_keyboard_gpt:main',
            'base_odom = teleop.base_odom:main',
        ],
    },
)
