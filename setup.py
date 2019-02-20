from distutils.core import setup
from Cython.Build import cythonize

setup(name='Path Planning GUI',
      ext_modules=cythonize(["robot.pyx", "obstacle.pyx", "gui.pyx"], annotate=True))