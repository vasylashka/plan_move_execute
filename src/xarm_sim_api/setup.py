from setuptools import find_packages, setup

package_name = 'xarm_sim_api'

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
            # Link the executable name 'xarm7_physics_sim' to the main function in the python file
            'xarm7_physics_sim = xarm_sim_api.xarm_pybullet_simulator:main',
        ],
    },
)
