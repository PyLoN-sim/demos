from glob import glob
from setuptools import setup, find_packages

name = 'pylon_demo_reusable'
setup(name=name, version='0.1.0', packages=find_packages(exclude=('test',)),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'README.md']),
                  *[('share/'+name+'/'+directory, glob(directory+'/'+pattern))
                    for directory, pattern in [('launch', '*.launch.py'), ('config', '*.yaml'), ('craft', '*.craft')]]],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='PyLoN', maintainer_email='user@example.com',
      description='Lifecycle-managed satellite deployment and powered landing demonstration.',
      entry_points={'console_scripts': ['mission = pylon_demo_reusable.node:main']})
