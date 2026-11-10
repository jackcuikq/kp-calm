from setuptools import find_packages, setup
from glob import glob
setup(name='a2r_robot', version='0.4.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/a2r_robot']),
                  ('share/a2r_robot', ['package.xml']),
                  ('share/a2r_robot/launch', glob('launch/*.py')),
                  ('share/a2r_robot/config', glob('config/*.yaml')),
                  ('share/a2r_robot/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='A2R Lab', maintainer_email='lab@example.invalid',
      url='https://a2r-lab.org',
      description='A2R robot hardware and safety nodes', license='MIT',
      entry_points={'console_scripts': ['hardware = a2r_robot.hardware:main',
                                       'safety = a2r_robot.safety:main',
                                       'visualization = a2r_robot.visualization:main']})
