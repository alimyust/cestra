from setuptools import find_packages, setup
import os, glob
package_name = 'cestra_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob.glob(os.path.join('worlds', '*.world'))),
        
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'cestra_turtle'), glob.glob(os.path.join('cestra_turtle', '*.py'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alimyust',
    maintainer_email='alimyust@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',    
    entry_points={
        'console_scripts': [
            'line_follower_classic = cestra_turtle.line_follower_classic:main',
            # 'line_follower_ai = cestra_turtle.line_follower_ai:main',
            'crop = cestra_turtle.vision_nodes:crop_main',

            'blur = cestra_turtle.vision_nodes:blur_main',

            'threshold = cestra_turtle.vision_nodes:threshold_main',

            'cleanup = cestra_turtle.vision_nodes:cleanup_main',

            'offset = cestra_turtle.vision_nodes:offset_main',

            'steering = cestra_turtle.vision_nodes:steering_main',
            'webcam = cestra_turtle.vision_nodes:webcam_main',


        ],
    },
)
