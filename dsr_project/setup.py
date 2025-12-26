from setuptools import find_packages, setup

package_name = 'dsr_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/board_pipeline.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jy',
    maintainer_email='jy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'brushing_action_client = dsr_project.brushing_action_client:main',
            'brushing_stop_server = dsr_project.brushing_stop_server:main',
            'cam_publisher = dsr_project.cam_publisher:main',
            'cam_subscriber = dsr_project.cam_subscriber:main',
            'Doma_a_pick_client = dsr_project.Doma_a_pick_client:main',
            'Doma_a_place_client = dsr_project.Doma_a_place_client:main',
            'Doma_a_pick_server = dsr_project.Doma_a_pick_server:main',
            'Doma_a_place_server = dsr_project.Doma_a_place_server:main',
            'doma_pick_stop_server = dsr_project.doma_pick_stop_server:main',
            'doma_pick_stop_server_1 = dsr_project.doma_pick_stop_server_1:main',
            'doma_place_stop_server = dsr_project.doma_place_stop_server:main',
            'doma_place_stop_server_1 = dsr_project.doma_place_stop_server_1:main',
            'erasing_stop_server = dsr_project.erasing_stop_server:main',
            'erasing_action_client = dsr_project.erasing_action_client:main',
            'firebase_publishing2 = dsr_project.firebase_publishing2:main',
            'main_node = dsr_project.main_node:main',
            'main_node_gem = dsr_project.main_node_gem:main',
            'oiling_action_client = dsr_project.oiling_action_client:main',
            'oiling_stop_server = dsr_project.oiling_stop_server:main',
            'test_gui = dsr_project.test_gui:main',
        ],
    },
)
