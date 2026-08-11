from glob import glob

from setuptools import find_packages, setup

package_name = 'voice_nav_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rapid_keywords_raw.txt']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edddddddddy',
    maintainer_email='983166955@qq.com',
    description='Local command interpretation and dialogue orchestration for VoiceNav Robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'agent_node = voice_nav_agent.agent_node:main',
            'rapid_mission_bridge = voice_nav_agent.rapid_mission_bridge:main',
            'rapid_initial_pose = voice_nav_agent.rapid_initial_pose:main',
            'rapid_cmd_vel_relay = voice_nav_agent.rapid_cmd_vel_relay:main',
            'rapid_voice_node = voice_nav_agent.rapid_voice_node:main',
            'rapid_web_console = voice_nav_agent.rapid_web_console_node:main',
        ],
    },
)
