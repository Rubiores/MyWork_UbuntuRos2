import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- SECCION MODIFICADA (SIN TILDES) ---
        
        # 1. Instalar archivos launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Instalar archivos urdf
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # 3. Instalar archivos de config (yaml y rviz)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
        # ---------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rubiores',
    maintainer_email='rubiores@todo.todo',
    description='Paquete de navegacion',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pico_odom_node = robot_nav.pico_odom_node:main',
            'lidar_node = robot_nav.lidar_node:main',
            'teleop_keyboard = robot_nav.teleop_keyboard:main',
            'motor_controller = robot_nav.motor_controller:main',
            'pico_bridge = robot_nav.pico_bridge:main',
            'simple_nav = robot_nav.simple_nav:main',
        ],
    },
)