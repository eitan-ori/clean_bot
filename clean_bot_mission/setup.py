from setuptools import setup
import os
from glob import glob

package_name = 'clean_bot_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
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
            
            # Legacy nodes (for backward compatibility)
            'mission_node = clean_bot_mission.mission:main',
            'exploration_node = clean_bot_mission.exploration:main',
            'coverage_mission = clean_bot_mission.coverage_mission:main',
        ],
    },
)
