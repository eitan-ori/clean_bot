from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'clean_bot_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    package_data={
        package_name: [
            'webapp/templates/*.html',
            'webapp/static/**/*',
            'webapp/saved_rooms/.gitkeep',
        ],
    },
    include_package_data=True,
    install_requires=['setuptools', 'numpy', 'scipy', 'flask', 'flask-socketio', 'pillow'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Autonomous exploration and coverage missions for Clean Bot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main mission controller (recommended)
            'full_mission = clean_bot_mission.full_mission:main',
            
            # Individual components (can run separately)
            'frontier_explorer = clean_bot_mission.frontier_explorer:main',
            'adaptive_coverage = clean_bot_mission.adaptive_coverage:main',
            'simple_coverage = clean_bot_mission.simple_coverage:main',
            
            # Web control panel
            'web_control = clean_bot_mission.webapp.app:main',
        ],
    },
)
