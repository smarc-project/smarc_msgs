from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  scripts=[	'scripts/sm_task_utils.py']
)

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['smarc_msgs'],
    package_dir={'': 'src'})

setup(**setup_args)