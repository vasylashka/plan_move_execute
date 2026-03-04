from setuptools import find_packages, setup

package_name = 'xarm_local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # The Planning Scene (Collision Environment)
            'xarm_planning_scene = xarm_local_planner.sim:main',

            # The Default Planner (Joint Space APF)
            'joint_space_apf_planner = xarm_local_planner.planners.default_planner:main',
        ],
    },
)
