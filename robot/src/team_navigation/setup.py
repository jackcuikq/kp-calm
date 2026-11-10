from glob import glob
from setuptools import find_packages, setup

setup(
    name='team_navigation', version='0.2.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/team_navigation']),
        ('share/team_navigation', ['package.xml']),
        ('share/team_navigation/launch', glob('launch/*.py')),
        ('share/team_navigation/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='A2R Lab', maintainer_email='lab@example.invalid',
    url='https://a2r-lab.org',
    description='Course navigation starter', license='MIT',
    entry_points={'console_scripts': ['navigation = team_navigation.navigation:main']},
)
