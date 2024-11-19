from setuptools import setup

package_name = 'turtlesim_gpt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'openai'],
    zip_safe=True,
    maintainer='keti',
    maintainer_email='your_email@example.com',
    description='GPT를 이용한 TurtleBot3 제어 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_gpt = turtlesim_gpt.turtlesim_controller:main',
            'turtlebot3_gpt = turtlesim_gpt.turtlebot3_controller:main'
        ],
    },
)