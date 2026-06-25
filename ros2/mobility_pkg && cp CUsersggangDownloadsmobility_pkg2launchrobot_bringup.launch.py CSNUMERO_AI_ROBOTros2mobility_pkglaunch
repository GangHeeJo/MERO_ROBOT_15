import os
from glob import glob  # 'form' 오타 수정 완료
from setuptools import find_packages, setup

package_name = 'mobility_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # [주의!] YOLO 엔진 파일을 위해 반드시 들어가야 하는 줄입니다 (추가 완료)
        (os.path.join('share', package_name, 'models'), glob('models/*.engine')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aiwinners',
    maintainer_email='aiwinners@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ugv_controller_node = mobility_pkg.ugv_controller_node:main',
            'camera_node = mobility_pkg.camera_node:main',
            'yolo_vision_node = mobility_pkg.yolo_vision_node:main',
            'main_decision_node = mobility_pkg.main_decision_node:main',
            'gripper_node = mobility_pkg.gripper_node:main',
        ],
    },
)