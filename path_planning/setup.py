from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
#comment
#next comment
d = generate_distutils_setup()
d['packages'] = ['path_planning']
d['package_dir'] = {'': 'src'}

setup(**d)
