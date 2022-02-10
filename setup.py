from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup  # type: ignore

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["open_manipulator_tenaci"], package_dir={"": "include"}
)
setup(**setup_args)
