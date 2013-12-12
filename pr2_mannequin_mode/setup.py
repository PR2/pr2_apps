from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pr2_mannequin_mode'],
    package_dir={'': 'src'},
    requires=['pr2_controllers_msgs', 'rospy', 'trajectory_msgs'],
    scripts=['scripts/trajectory_lock.py']
)

setup(**d)
