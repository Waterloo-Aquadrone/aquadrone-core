from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['thrust_computer', 'thrust_distributor', 'real_thruster_types']
d['package_dir'] = {'': 'src'}

setup(**d)
