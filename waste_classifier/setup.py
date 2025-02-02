import glob
from setuptools import setup
import os

package_name = 'my_robot'

# Use glob correctly to collect all files
launch_files = glob.glob('launch/*')
urdf_files = glob.glob('urdf/*')
config_files = glob.glob('config/*')
mesh_files = glob.glob('mesh/*')
world_files = glob.glob('world/*')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'urdf'), urdf_files),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'mesh'), mesh_files),
        (os.path.join('share', package_name, 'world'), world_files),
    ],
    install_requires=[
        'setuptools',
        'opencv-python==4.7.0.72',
        'tensorflow==2.9.0',
        'keras'],
    zip_safe=True,
    maintainer='aderajew',
    maintainer_email='adeyeshi294@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator = my_robot.trajectory_gen:main',
            'inverse_kinematics = my_robot.inverse_kinematics_solution:main',
            'waste_server = my_robot.waste_server:main',
            'waste_client = my_robot.waste_client:main',
            'image_save = my_robot.image_save:main',
            'move = my_robot.movement:main',
            'check = my_robot.test_move:main',
            'test = my_robot.gazebo:main',
        ],
    },
)
